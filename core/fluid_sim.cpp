#include "fluid_sim.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace fluid {

FluidSimulator::FluidSimulator()
    : particle_count_(0), led_count_(0) {
    std::memset(grid_, 0, sizeof(grid_));
    std::memset(is_led_mask_, 0, sizeof(is_led_mask_));
    init_led_layout();
    reset();
}

void FluidSimulator::init_led_layout() {
    led_count_ = 0;
    const float half_dim = (Config::GRID_DIM - 1.0f) * 0.5f;
    const float divisor = 8.65f; // Дає РІВНО 240 симетричних світлодіодів

    for (uint8_t y = 0; y < Config::GRID_DIM; ++y) {
        for (uint8_t x = 0; x < Config::GRID_DIM; ++x) {
            float nx = (static_cast<float>(x) - half_dim) / divisor;
            float ny = (static_cast<float>(y) - half_dim) / divisor;

            // Перевіряємо коло кулона
            if ((nx * nx + ny * ny) <= 1.0f) {
                is_led_mask_[y][x] = true;
                if (led_count_ < Config::TOTAL_LEDS) {
                    leds_[led_count_].pos = Vec2(nx, ny);
                    leds_[led_count_].grid_x = x;
                    leds_[led_count_].grid_y = y;
                    leds_[led_count_].brightness = 0;
                    ++led_count_;
                }
            } else {
                is_led_mask_[y][x] = false;
            }
        }
    }
}

void FluidSimulator::reset() {
    set_fill_level(0.60f);
}

void FluidSimulator::set_fill_level(float ratio) {
    if (ratio < 0.10f) ratio = 0.10f;
    if (ratio > 1.00f) ratio = 1.00f;

    particle_count_ = 0;

    // Калібровані кількості частинок для точних відсотків матриці:
    // 20% -> 48 LED, 40% -> 96 LED, 60% -> 144 LED, 80% -> 192 LED, 100% -> 240 LED
    uint16_t target_count = 140;
    if (ratio <= 0.25f) target_count = 12;
    else if (ratio <= 0.45f) target_count = 27;
    else if (ratio <= 0.65f) target_count = 46;
    else if (ratio <= 0.85f) target_count = 76;
    else target_count = 140;

    const float spacing = 0.128f;
    for (float y = 0.95f; y >= -0.95f && particle_count_ < target_count; y -= spacing) {
        for (float x = -0.95f; x <= 0.95f && particle_count_ < target_count; x += spacing) {
            if (x * x + y * y <= 0.95f) {
                particles_[particle_count_].pos = Vec2(x, y);
                particles_[particle_count_].vel = Vec2(0.0f, 0.0f);
                ++particle_count_;
            }
        }
    }

    // Швидке розправлення поверхні рідини для точного стартового відліку діодів
    for (int i = 0; i < 6; ++i) {
        resolve_particle_collisions();
        apply_boundary_constraints();
    }

    render_to_leds();
}

void FluidSimulator::adjust_particle_count(int delta) {
    if (delta > 0) {
        // Додаємо частинки зверху існуючих
        const float spacing = Config::PARTICLE_RADIUS * 1.15f;
        for (float y = 0.2f; y >= -0.8f && delta > 0 && particle_count_ < Config::MAX_PARTICLES; y -= spacing) {
            for (float x = -0.7f; x <= 0.7f && delta > 0 && particle_count_ < Config::MAX_PARTICLES; x += spacing) {
                if (x * x + y * y <= 0.75f) {
                    particles_[particle_count_].pos = Vec2(x, y);
                    particles_[particle_count_].vel = Vec2(0.0f, 0.0f);
                    ++particle_count_;
                    --delta;
                }
            }
        }
    } else if (delta < 0) {
        int remove_count = -delta;
        if (particle_count_ > static_cast<uint16_t>(remove_count + 5)) {
            particle_count_ -= remove_count;
        }
    }

    render_to_leds();
}

void FluidSimulator::add_impulse(Vec2 impulse) {
    for (uint16_t i = 0; i < particle_count_; ++i) {
        particles_[i].vel += impulse;
    }
}

void FluidSimulator::add_rotational_impulse(float omega) {
    if (std::abs(omega) < 1e-4f) return;
    for (uint16_t i = 0; i < particle_count_; ++i) {
        float r = particles_[i].pos.length();
        if (r > 0.25f) {
            Vec2 v_wall(omega * particles_[i].pos.y, -omega * particles_[i].pos.x);
            float coupling = ((r - 0.25f) / 0.72f) * 0.35f;
            if (coupling > 0.40f) coupling = 0.40f;
            particles_[i].vel += (v_wall - particles_[i].vel) * coupling;
        }
    }
}

void FluidSimulator::step(Vec2 gravity, float dt, float angular_velocity) {
    if (dt <= 0.0f) return;

    // Обмеження максимального кроку часу для уникнення нестабільності
    if (dt > 0.033f) dt = 0.033f;

    // Зберігаємо початкові позиції для виведення швидкості за методом PBF (Macklin & Müller)
    Vec2 old_pos[Config::MAX_PARTICLES];

    // 1. Передбачення позицій (Predict positions під дією сил)
    for (uint16_t i = 0; i < particle_count_; ++i) {
        old_pos[i] = particles_[i].pos;

        particles_[i].vel *= Config::DAMPING;
        particles_[i].vel += gravity * dt;

        if (std::abs(angular_velocity) > 0.01f) {
            float r = particles_[i].pos.length();
            if (r > 0.25f) {
                Vec2 v_wall(angular_velocity * particles_[i].pos.y, -angular_velocity * particles_[i].pos.x);
                float coupling = ((r - 0.25f) / 0.72f) * 0.35f;
                if (coupling > 0.40f) coupling = 0.40f;
                particles_[i].vel += (v_wall - particles_[i].vel) * coupling;
            }
        }

        particles_[i].pos += particles_[i].vel * dt;
    }

    // 2. Ітеративне розв'язання геометричних обмежень (Constraint Solving)
    for (int iter = 0; iter < 4; ++iter) {
        resolve_particle_collisions();
        apply_boundary_constraints();
    }

    // 3. Оновлення швидкості за канонічною формулою PBF: v = (pos_new - pos_old) / dt
    // Це природно перетворює вертикальний удар у горизонтальне розтікання (сплеск),
    // але автоматично затухає в нуль у стані спокою (без вічного двигуна)!
    float inv_dt = 1.0f / dt;
    for (uint16_t i = 0; i < particle_count_; ++i) {
        particles_[i].vel = (particles_[i].pos - old_pos[i]) * inv_dt;
        particles_[i].vel *= Config::DAMPING;
    }

    // 4. XSPH В'язкість за стандартом PBF (c <= 0.01..0.02)
    const float visc_radius = Config::PARTICLE_RADIUS * 2.2f;
    const float visc_radius_sq = visc_radius * visc_radius;
    const float c_visc = 0.015f; // Наукове каліброване значення в'язкості
    for (uint16_t i = 0; i < particle_count_; ++i) {
        for (uint16_t j = i + 1; j < particle_count_; ++j) {
            Vec2 delta = particles_[i].pos - particles_[j].pos;
            float dist_sq = delta.length_sq();
            if (dist_sq < visc_radius_sq && dist_sq > 1e-6f) {
                float dist = std::sqrt(dist_sq);
                float q = 1.0f - (dist / visc_radius);
                Vec2 rel_vel = particles_[j].vel - particles_[i].vel;
                Vec2 visc_impulse = rel_vel * (c_visc * q);
                particles_[i].vel += visc_impulse;
                particles_[j].vel -= visc_impulse;
            }
        }
    }

    // 5. Оновлення відображення на світлодіодах
    render_to_leds();
}

void FluidSimulator::resolve_particle_collisions() {
    const float min_dist = Config::PARTICLE_RADIUS * 2.0f;
    const float min_dist_sq = min_dist * min_dist;

    for (uint16_t i = 0; i < particle_count_; ++i) {
        for (uint16_t j = i + 1; j < particle_count_; ++j) {
            Vec2 delta = particles_[i].pos - particles_[j].pos;
            float dist_sq = delta.length_sq();

            if (dist_sq < min_dist_sq && dist_sq > 1e-6f) {
                float dist = std::sqrt(dist_sq);
                float overlap = min_dist - dist;
                Vec2 normal = delta / dist;

                // Нестисливість рідини: розсунення позицій
                Vec2 push = normal * (0.5f * overlap * Config::REPULSION_STRENGTH);
                particles_[i].pos += push;
                particles_[j].pos -= push;
            }
        }
    }
}

void FluidSimulator::apply_boundary_constraints() {
    const float max_radius = 0.97f;

    for (uint16_t i = 0; i < particle_count_; ++i) {
        float dist = particles_[i].pos.length();

        if (dist > max_radius) {
            Vec2 normal = particles_[i].pos / (dist > 1e-6f ? dist : 1.0f);
            particles_[i].pos = normal * max_radius;
        }
    }
}

void FluidSimulator::render_to_leds() {
    std::memset(grid_, 0, sizeof(grid_));

    // Радіус впливу частинки на світлодіод (2.0 для суцільного заповнення всіх 240 діодів)
    const float led_influence_radius = Config::PARTICLE_RADIUS * 2.0f;
    const float influence_sq = led_influence_radius * led_influence_radius;

    for (uint16_t k = 0; k < led_count_; ++k) {
        float accumulated_density = 0.0f;
        const Vec2& led_pos = leds_[k].pos;

        for (uint16_t i = 0; i < particle_count_; ++i) {
            float dist_sq = (led_pos - particles_[i].pos).length_sq();
            if (dist_sq < influence_sq) {
                float dist = std::sqrt(dist_sq);
                // Згладжуюче ядро сплайна (Smoothstep)
                float q = 1.0f - (dist / led_influence_radius);
                accumulated_density += q * q;
            }
        }

        // Неперервний плавний градієнт для багатих відтінків синього кольору
        float b_val = accumulated_density * 320.0f;
        if (b_val > 255.0f) b_val = 255.0f;

        uint8_t brightness = (accumulated_density > 0.015f) ? static_cast<uint8_t>(b_val) : 0;
        leds_[k].brightness = brightness;

        uint8_t gx = leds_[k].grid_x;
        uint8_t gy = leds_[k].grid_y;
        if (gx < Config::GRID_DIM && gy < Config::GRID_DIM) {
            grid_[gy][gx] = brightness;
        }
    }
}

} // namespace fluid
