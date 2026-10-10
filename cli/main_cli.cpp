#include "../core/fluid_sim.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

enum KeyAction {
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_ROTATE_TOGGLE,
    KEY_SPIN_CW,
    KEY_SPIN_CCW,
    KEY_SPEED_UP,
    KEY_SPEED_DOWN,
    KEY_ADD_PARTICLES,
    KEY_SUB_PARTICLES,
    KEY_LEVEL_1,
    KEY_LEVEL_2,
    KEY_LEVEL_3,
    KEY_LEVEL_4,
    KEY_LEVEL_5,
    KEY_SHAKE,
    KEY_RESET,
    KEY_QUIT
};

class NonBlockingInput {
public:
    NonBlockingInput() {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    ~NonBlockingInput() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    KeyAction read_key() {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) <= 0) {
            return KEY_NONE;
        }

        if (c == 27) { // Escape sequence (стрілочки)
            char seq[2] = {0, 0};
            if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0) {
                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A': return KEY_UP;
                        case 'B': return KEY_DOWN;
                        case 'C': return KEY_RIGHT;
                        case 'D': return KEY_LEFT;
                    }
                }
            }
            return KEY_NONE;
        }

        switch (c) {
            case 'w': case 'W': return KEY_UP;
            case 's': case 'S': return KEY_DOWN;
            case 'a': case 'A': return KEY_LEFT;
            case 'd': case 'D': return KEY_RIGHT;
            case ' ': return KEY_ROTATE_TOGGLE;
            case 'r': case 'R': return KEY_SPIN_CW;
            case 'e': case 'E': return KEY_SPIN_CCW;
            case 'f': case 'F': return KEY_SPEED_UP;
            case 'v': case 'V': return KEY_SPEED_DOWN;
            case '+': case '=': return KEY_ADD_PARTICLES;
            case '-': case '_': return KEY_SUB_PARTICLES;
            case '1': return KEY_LEVEL_1;
            case '2': return KEY_LEVEL_2;
            case '3': return KEY_LEVEL_3;
            case '4': return KEY_LEVEL_4;
            case '5': return KEY_LEVEL_5;
            case 'x': case 'X': return KEY_SHAKE;
            case 'c': case 'C': return KEY_RESET;
            case 'q': case 'Q': return KEY_QUIT;
            default: return KEY_NONE;
        }
    }

private:
    struct termios orig_termios_;
};

void run_benchmark() {
    std::cout << "\n========================================\n";
    std::cout << "  FLUID SIMULATION BENCHMARK (C++ CORE)  \n";
    std::cout << "========================================\n";

    fluid::FluidSimulator sim;
    const int total_frames = 1000;
    const float dt = 0.016f; // 60 FPS delta

    std::cout << "Кількість активних частинок: " << sim.get_particle_count() << "\n";
    std::cout << "Кількість світлодіодів на кулоні: " << sim.get_led_count() << "\n";
    std::cout << "Виконується " << total_frames << " кадрів симуляції...\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < total_frames; ++i) {
        float angle = i * 0.02f;
        fluid::Vec2 dynamic_g(std::sin(angle) * 9.8f, std::cos(angle) * 9.8f);
        sim.step(dynamic_g, dt);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;

    double avg_frame_ms = elapsed_ms.count() / total_frames;
    double max_fps = 1000.0 / avg_frame_ms;

    std::cout << "\nРЕЗУЛЬТАТИ ТЕСТУ:\n";
    std::cout << "Загальний час (" << total_frames << " кадрів): " << elapsed_ms.count() << " ms\n";
    std::cout << "Середній час на 1 крок фізики + LED рендер: " << avg_frame_ms << " ms\n";
    std::cout << "Теоретична пікова продуктивність: " << static_cast<int>(max_fps) << " FPS\n";
    std::cout << "========================================\n\n";
}

void print_led_matrix(const fluid::FluidSimulator& sim, int frame_idx, const std::string& action, bool auto_rotate, float rotate_speed) {
    // Очищення екрана та повернення курсору у верхній лівий кут
    std::cout << "\033[H\033[J";

    std::cout << "\033[1;36m=== РОЗУМНИЙ КУЛОН: СИМУЛЯЦІЯ ВОДИ (C++ Core) ===\033[0m\n";
    std::cout << "Світлодіодів: \033[1;33m" << sim.get_led_count() 
              << "\033[0m | Частинок води: \033[1;32m" << sim.get_particle_count() 
              << "\033[0m | Кадр: " << frame_idx << "\n";
    
    std::cout << "Стан: ";
    if (auto_rotate) {
        std::cout << "\033[1;35mАВТО-ОБЕРТАННЯ ↻ [швидкість: " << static_cast<int>(rotate_speed * 500) << "%]\033[0m";
    } else {
        std::cout << "\033[1;32m" << action << "\033[0m | Гравітація Землі: \033[1;36mТЯГНЕ НА ДНО ↓\033[0m";
    }
    std::cout << "\n";

    std::cout << "\033[1;37mКерування (динаміка хвиль та інерція):\033[0m\n";
    std::cout << "  • Крутнути кулон (вихор!): \033[1;33m[R]\033[0m за стрілкою ↻ | \033[1;33m[E]\033[0m проти ↺ | \033[1;33m[Space]\033[0m авто\n";
    std::cout << "  • Швидкість обертання:     \033[1;33m[F]\033[0m швидше (+) | \033[1;33m[V]\033[0m повільніше (-)\n";
    std::cout << "  • Хвилі/Поштовх:           \033[1;33m[← / A]\033[0m вліво | \033[1;33m[→ / D]\033[0m вправо | \033[1;33m[↑ / W]\033[0m сплеск | \033[1;33m[↓ / S]\033[0m осідання\n";
    std::cout << "  • Рівень води:             \033[1;33m[1, 2, 3, 4, 5]\033[0m (20%..100%) | \033[1;33m[+ / -]\033[0m частинки\n";
    std::cout << "  • Струс: \033[1;33m[X]\033[0m | Скинути: \033[1;33m[C]\033[0m | Вихід: \033[1;33m[Q]\033[0m\n\n";

    // Відмальовуємо круглу матрицю 18x18 (Рівно 240 світлодіодів)
    for (uint8_t y = 0; y < fluid::Config::GRID_DIM; ++y) {
        std::cout << "   ";
        for (uint8_t x = 0; x < fluid::Config::GRID_DIM; ++x) {
            if (!sim.is_led(x, y)) {
                // Поза круглим кулоном
                std::cout << "  ";
            } else {
                uint8_t b = sim.get_grid_pixel(x, y);
                if (b == 0) {
                    // Згаслий світлодіод (повітря)
                    std::cout << "\033[38;5;238m· \033[0m";
                } else if (b < 60) {
                    // Темно-синій (краї меніска, бризки)
                    std::cout << "\033[38;5;27m░░\033[0m";
                } else if (b < 125) {
                    // Насичений сапфіровий синій (хвилі)
                    std::cout << "\033[38;5;33m▒▒\033[0m";
                } else if (b < 195) {
                    // Лазурний морський синій (вода)
                    std::cout << "\033[38;5;39m██\033[0m";
                } else {
                    // Сяючий електричний ціан (глибина / товща)
                    std::cout << "\033[38;5;81m██\033[0m";
                }
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n\033[2m(Вода завжди хлюпається і стікає назад на дно під силою тяжіння)\033[0m\n";
    std::cout.flush();
}

void run_interactive_demo() {
    fluid::FluidSimulator sim;
    NonBlockingInput input;

    bool auto_rotate = false;
    float auto_angle = 0.0f;
    float rotate_speed = 0.12f; // Базова швидкість обертання (~0.75 об/сек)
    fluid::Vec2 dynamic_acc(0.0f, 0.0f); // Тимчасовий імпульс прискорення від руху
    std::string action = "Вода спокійно лежить на дні кулона";
    int frame = 0;

    std::cout << "\033[?25l"; // Приховуємо курсор терміналу

    while (true) {
        KeyAction act = input.read_key();
        if (act == KEY_QUIT) {
            break;
        } else if (act == KEY_ROTATE_TOGGLE) {
            auto_rotate = !auto_rotate;
            action = auto_rotate ? "Авто-обертання" : "Вода стікає на дно";
        } else if (act == KEY_SPEED_UP) {
            rotate_speed = std::min(0.35f, rotate_speed + 0.03f);
            action = "Прискорено обертання (+) [швидкість: " + std::to_string(static_cast<int>(rotate_speed * 500)) + "%]";
        } else if (act == KEY_SPEED_DOWN) {
            rotate_speed = std::max(0.03f, rotate_speed - 0.03f);
            action = "Сповільнено обертання (-) [швидкість: " + std::to_string(static_cast<int>(rotate_speed * 500)) + "%]";
        } else if (act == KEY_SPIN_CW) {
            auto_rotate = false;
            sim.add_rotational_impulse(6.0f);
            action = "Крутнули кулон ↻↻ (потужний вихор і хвилі!)";
        } else if (act == KEY_SPIN_CCW) {
            auto_rotate = false;
            sim.add_rotational_impulse(-6.0f);
            action = "Крутнули кулон ↺↺ (потужний вихор і хвилі!)";
        } else if (act == KEY_LEFT) {
            auto_rotate = false;
            // Потужна хвиля вліво (підіймається вгору по дузі кулона)
            sim.add_impulse(fluid::Vec2(-4.2f, -1.0f));
            dynamic_acc.x -= 32.0f;
            action = "Поштовх ВЛІВО ◄◄ (хвиля накочується й коливається!)";
        } else if (act == KEY_RIGHT) {
            auto_rotate = false;
            // Потужна хвиля вправо (підіймається вгору по дузі кулона)
            sim.add_impulse(fluid::Vec2(4.2f, -1.0f));
            dynamic_acc.x += 32.0f;
            action = "Поштовх ВПРАВО ►► (хвиля накочується й коливається!)";
        } else if (act == KEY_UP) {
            auto_rotate = false;
            // Сплеск / стрибок вгору
            sim.add_impulse(fluid::Vec2(0.0f, -5.5f));
            dynamic_acc.y -= 35.0f;
            action = "Стрибок / Сплеск вгору ▲▲ (вода падає на дно)";
        } else if (act == KEY_DOWN) {
            auto_rotate = false;
            // Присідання / осідання
            sim.add_impulse(fluid::Vec2(0.0f, 4.0f));
            dynamic_acc.y += 24.0f;
            action = "Різке осідання на дно ▼";
        } else if (act == KEY_SHAKE) {
            auto_rotate = false;
            float rx = (std::rand() % 40 - 20) * 0.25f;
            float ry = (std::rand() % 40 - 20) * 0.25f;
            sim.add_impulse(fluid::Vec2(rx, ry));
            dynamic_acc.x += rx * 10.0f;
            dynamic_acc.y += ry * 10.0f;
            action = "Струс кулона ⚡";
        } else if (act == KEY_ADD_PARTICLES) {
            sim.adjust_particle_count(8);
        } else if (act == KEY_SUB_PARTICLES) {
            sim.adjust_particle_count(-8);
        } else if (act == KEY_LEVEL_1) {
            sim.set_fill_level(0.20f);
            action = "Рівень: 20% води (48 LED)";
        } else if (act == KEY_LEVEL_2) {
            sim.set_fill_level(0.40f);
            action = "Рівень: 40% води (96 LED)";
        } else if (act == KEY_LEVEL_3) {
            sim.set_fill_level(0.60f);
            action = "Рівень: 60% води (144 LED)";
        } else if (act == KEY_LEVEL_4) {
            sim.set_fill_level(0.80f);
            action = "Рівень: 80% води (192 LED)";
        } else if (act == KEY_LEVEL_5) {
            sim.set_fill_level(1.00f);
            action = "Рівень: 100% повний (240 LED)";
        } else if (act == KEY_RESET) {
            auto_rotate = false;
            dynamic_acc = fluid::Vec2(0.0f, 0.0f);
            sim.reset();
            action = "Скидання стану (вода на дні)";
        }

        // Обчислення сили: Земне тяжіння ЗАВЖДИ тягне вниз (+9.8f),
        // а динамічне прискорення від поштовхів згасає поступово!
        fluid::Vec2 total_g;
        float current_omega = 0.0f;
        if (auto_rotate) {
            auto_angle += rotate_speed;
            current_omega = rotate_speed / 0.025f; // кутова швидкість у рад/с
            total_g = fluid::Vec2(std::sin(auto_angle) * 9.8f, std::cos(auto_angle) * 9.8f);
        } else {
            total_g = fluid::Vec2(dynamic_acc.x, 9.8f + dynamic_acc.y);

            // Плавне згасання (0.91f замість 0.82f) — хвилі коливаються туди-сюди 4-6 циклів!
            dynamic_acc.x *= 0.93f;
            dynamic_acc.y *= 0.93f;
            if (std::abs(dynamic_acc.x) < 0.2f) dynamic_acc.x = 0.0f;
            if (std::abs(dynamic_acc.y) < 0.2f) dynamic_acc.y = 0.0f;
        }

        sim.step(total_g, 0.025f, current_omega);
        print_led_matrix(sim, frame++, action, auto_rotate, rotate_speed);

        std::this_thread::sleep_for(std::chrono::milliseconds(25)); // ~40 FPS
    }

    std::cout << "\033[?25h\033[0m\n"; // Відновлюємо курсор
}

int main(int argc, char* argv[]) {
    if (argc > 1 && std::string(argv[1]) == "--benchmark") {
        run_benchmark();
        return 0;
    }

    std::cout << "Запуск симулятора кулона...\n";
    std::cout << "Для запуску бенчмарку використовуйте: ./build/fluid_sim_cli --benchmark\n\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    run_interactive_demo();
    return 0;
}
