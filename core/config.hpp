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

    // В'язкість / збереження кінетичної енергії на кожному кроці (0.995f для бурхливого тривалого хлюпання хвиль)
    static constexpr float DAMPING = 0.995f;

    // Коефіцієнт пружності при ударі об стінку кулона (0.55f для відбиття високих хвиль)
    static constexpr float WALL_BOUNCE = 0.55f;

    // Сила розслаблення тиску (0.85f забезпечує пласку горизонтальну поверхню без кристалічного склеювання)
    static constexpr float REPULSION_STRENGTH = 0.85f;

    // Розмірність квадратної сітки для матриці (18x18 з круговою маскою дає ~240 активних LED)
    static constexpr uint8_t GRID_DIM = 18;

    // Загальна кількість світлодіодів кулона
    static constexpr uint16_t TOTAL_LEDS = 240;
};

} // namespace fluid
