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

    // Розмірність квадратної сітки для матриці (18x18 з круговою маскою дає ~240 активних LED)
    static constexpr uint8_t GRID_DIM = 18;

    // Загальна кількість світлодіодів кулона
    static constexpr uint16_t TOTAL_LEDS = 240;
};

} // namespace fluid
