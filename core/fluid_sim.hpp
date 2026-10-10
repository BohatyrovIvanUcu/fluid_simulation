#pragma once

#include "vec2.hpp"
#include "config.hpp"
#include <cstdint>

namespace fluid {

// Структура фізичної частинки рідини
struct Particle {
    Vec2 pos;       // Поточна координата [-1.0, 1.0]
    Vec2 vel;       // Вектор швидкості
};

// Структура опису світлодіода на круглій платі кулона
struct LedPixel {
    Vec2 pos;       // Фізична координата LED на платі [-1.0, 1.0]
    uint8_t grid_x; // Координата X у віртуальній матриці
    uint8_t grid_y; // Координата Y у віртуальній матриці
    uint8_t brightness; // Рівень яскравості (0 = вимкнено, 255 = максимальна яскравість)
};

class FluidSimulator {
public:
    FluidSimulator();

    // Ініціалізація початкового стану (розташування води на дні кулона)
    void reset();

    // Встановлення рівня заповненості кулона водою (від 0.1f = 10% до 1.0f = 100%)
    void set_fill_level(float ratio);

    // Додати / зменшити кількість частинок
    void adjust_particle_count(int delta);

    // Додати фізичний імпульс/поштовх рідині (імітація кроку, струсу чи руху)
    void add_impulse(Vec2 impulse);

    // Додати обертальний імпульс кулона (закручування рідини у вихор)
    // omega: кутова швидкість обертання кулона в рад/с (+ за годинниковою, - проти)
    void add_rotational_impulse(float omega);

    // Крок фізичної симуляції
    // gravity: вектор прискорення від акселерометра (наприклад, (0, 9.8f))
    // dt: дельта часу в секундах (зазвичай 1/40s..1/60s)
    // angular_velocity: кутова швидкість обертання кулона (рад/с)
    void step(Vec2 gravity, float dt, float angular_velocity = 0.0f);

    // Растеризація рідини у світлодіоди кулона
    void render_to_leds();

    // Отримання доступу до частинок (для налагодження чи детальної візуалізації)
    const Particle* get_particles() const { return particles_; }
    uint16_t get_particle_count() const { return particle_count_; }

    // Отримання масиву світлодіодів кулона
    const LedPixel* get_leds() const { return leds_; }
    uint16_t get_led_count() const { return led_count_; }

    // Перевірка, чи є точка (x, y) фізичним світлодіодом кулона
    bool is_led(uint8_t x, uint8_t y) const {
        if (x < Config::GRID_DIM && y < Config::GRID_DIM) {
            return is_led_mask_[y][x];
        }
        return false;
    }

    // Отримання віртуальної 2D сітки [18][18] (0 = порожньо, 255 = максимальна яскравість)
    uint8_t get_grid_pixel(uint8_t x, uint8_t y) const {
        if (x < Config::GRID_DIM && y < Config::GRID_DIM) {
            return grid_[y][x];
        }
        return 0;
    }

private:
    void init_led_layout();
    void resolve_particle_collisions();
    void apply_boundary_constraints();

    Particle particles_[Config::MAX_PARTICLES];
    uint16_t particle_count_;

    LedPixel leds_[Config::TOTAL_LEDS];
    uint16_t led_count_;

    uint8_t grid_[Config::GRID_DIM][Config::GRID_DIM];
    bool is_led_mask_[Config::GRID_DIM][Config::GRID_DIM];
};

} // namespace fluid
