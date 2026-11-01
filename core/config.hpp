#pragma once

#include <cstdint>

namespace fluid {

// Фізичні та математичні константи симулятора
struct Config {
    // Радіус круглого кулона (нормалізований до 1.0f)
    static constexpr float PENDANT_RADIUS = 1.0f;

    // Максимальна кількість частинок (для 100% заповнення всіх 240 LED)
    static constexpr uint16_t MAX_PARTICLES = 160;

    // Ефективний радіус взаємодії частинки (ядро згладжування / радіус виштовхування)
    static constexpr float PARTICLE_RADIUS = 0.12f;

    // В'язкість / збереження кінетичної енергії на кожному кроці (0.988f для природного затухання хвиль)
    static constexpr float DAMPING = 0.988f;

    // Коефіцієнт пружності при ударі об стінку кулона (0.38f для реалістичного відбиття)
    static constexpr float WALL_BOUNCE = 0.38f;

    // Сила розслаблення тиску (0.92f забезпечує пласку горизонтальну поверхню води)
    static constexpr float REPULSION_STRENGTH = 0.92f;

    // Розмірність фізичної матриці кулона (16 стовпчиків x 15 рядків, 216 фізичних LED із 240 логічних)
    static constexpr uint8_t GRID_COLS = 16;
    static constexpr uint8_t GRID_ROWS = 15;
    static constexpr uint8_t GRID_DIM = 16;

    // Загальна кількість фізичних світлодіодів на круглій платі (36 + 144 + 36 = 216 LED: D04..D237)
    static constexpr uint16_t TOTAL_LEDS = 216;
};

} // namespace fluid
