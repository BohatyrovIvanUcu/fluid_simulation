#include <Arduino.h>
#include <cmath>
#include <cstring>

// ==============================================================================
// AUTONOMOUS INTERACTIVE FLUID SIMULATION PENDANT — STM32 FIRMWARE (STABLE BACKUP)
// ==============================================================================
// Повна інтеграція:
// 1. Фізичний рушій Position-Based Fluids (Macklin & Müller) + XSPH в'язкість
//    із вашого core/fluid_sim.cpp (нуль динамічних алокацій, O(1) пам'ять).
// 2. Драйвер круглої Charlieplexing-матриці 16x15 (216 фізичних LED з 240)
//    із точною бієктивною картою пінів та високочастотним POV без мерехтіння.
// 3. Безконфліктний драйвер MPU6050 на D5 (PB6) та D4 (PB7) із захистом від
//    хибних I2C START-умов під час сканування матриці (без засвітки D81/D82
//    та без фантомних діагоналей).
// ==============================================================================

// ------------------------------------------------------------------------------
// 1. КОНФІГУРАЦІЯ МАТРИЦІ CHARLIEPLEXING (16 ЛІНІЙ -> 216 / 240 LED)
// ------------------------------------------------------------------------------
const int NUM_LINES = 16;
const int linePins[NUM_LINES] = {
    PB3,  // L[0]  (D13)
    PA4,  // L[1]  (A3)
    PA6,  // L[2]  (A5, паралельно PB6/D5 через SB16)
    PA8,  // L[3]  (D9)
    PA10, // L[4]  (D0)
    PB5,  // L[5]  (D11)
    PB0,  // L[6]  (D3)
    PB1,  // L[7]  (D6)
    PB4,  // L[8]  (D12)
    PA11, // L[9]  (D10)
    PA9,  // L[10] (D1)
    PA7,  // L[11] (A6)
    PA5,  // L[12] (A4, паралельно PB7/D4 через SB18)
    PA3,  // L[13] (A2)
    PA1,  // L[14] (A1)
    PA0   // L[15] (A0)
};

// Межі фізично присутніх світлодіодів у кожному з 15 рядків (кругла форма плати)
const uint8_t rowStart[15] = { 3,  2,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3};
const uint8_t rowEnd[15]   = {12, 13, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 12};

// Переведення всіх ліній у Hi-Z + компенсація підтяжок I2C на PA5/PB7 та PA6/PB6
inline void clearAll() {
    for (int i = 0; i < NUM_LINES; i++) {
        pinMode(linePins[i], INPUT);
    }
    pinMode(PA6, INPUT_PULLDOWN);
    pinMode(PB6, INPUT_PULLDOWN);
    pinMode(PA5, INPUT_PULLDOWN);
    pinMode(PB7, INPUT_PULLDOWN);
}

// Точна бієктивна формула (X, Y) -> (Anode, Cathode) для всіх 240 позицій
inline void getLedPins(int x, int y, int &anodePin, int &cathodePin) {
    int aIdx, cIdx;
    if (((x + y) & 1) == 0) {
        cIdx = (((x + y) >> 1) - 1 + 16) & 15;
        aIdx = (cIdx - (y + 1) + 16) & 15;
    } else if (x == 15) {
        cIdx = ((y >> 1) + 7) & 15;
        aIdx = (cIdx - (y + 1) + 16) & 15;
    } else {
        aIdx = (((x + y - 1) >> 1) + 16) & 15;
        cIdx = (aIdx - (y + 1) + 16) & 15;
    }
    anodePin = linePins[aIdx];
    cathodePin = linePins[cIdx];
}

// Піни шини I2C акселерометра MPU6050 (автоматично уточнюються при старті в detectAndInitImu)
namespace ImuPins {
    uint8_t sdaLine = PA5; // Лінія матриці, з'єднана з SDA (PA5 для D4/PB7, або PA6 для D5/PB6)
    uint8_t sclLine = PA6; // Лінія матриці, з'єднана з SCL (PA6 для D5/PB6, або PA5 для D4/PB7)
    uint8_t sdaMcu  = PB7; // Пін Nucleo SDA (D4 = PB7 або D5 = PB6)
    uint8_t sclMcu  = PB6; // Пін Nucleo SCL (D5 = PB6 або D4 = PB7)
}

// Безконфліктний імпульс одного світлодіода:
// Захищає шину I2C (SCL/SDA) від хибної умови START (переходу SDA HIGH->LOW при SCL == HIGH),
// щоб MPU6050 ніколи не затискав лінію SDA в нуль і не засвічував D81/D82 чи фантомні діагоналі!
inline void pulseLedSafe(uint8_t aPin, uint8_t cPin, uint8_t us) {
    const uint8_t sdaLine = ImuPins::sdaLine;
    const uint8_t sclMcu  = ImuPins::sclMcu;

    if (cPin == sdaLine) {
        // Перед тим як опустити SDA в LOW, коротко притискаємо SCL в LOW,
        // щоб спад SDA відбувся при SCL == 0 (це НЕ є умовою I2C START для MPU6050!)
        digitalWrite(sclMcu, LOW);
        pinMode(sclMcu, OUTPUT);
        digitalWrite(cPin, LOW);
        pinMode(cPin, OUTPUT);
        pinMode(sclMcu, INPUT_PULLDOWN);

        digitalWrite(aPin, HIGH);
        pinMode(aPin, OUTPUT);

        delayMicroseconds(us);

        // Спочатку гасимо анод (якщо це SCL — він падає в LOW першим), потім відпускаємо SDA
        digitalWrite(aPin, LOW);
        pinMode(aPin, (aPin == PA5 || aPin == PA6) ? INPUT_PULLDOWN : INPUT);
        pinMode(cPin, INPUT_PULLDOWN);
    } else if (aPin == sdaLine) {
        digitalWrite(cPin, LOW);
        pinMode(cPin, OUTPUT);
        digitalWrite(aPin, HIGH);
        pinMode(aPin, OUTPUT);

        delayMicroseconds(us);

        // Відключаємо драйвер 3.3V на PA5 (SDA) без вмикання PB6 (SCL) у LOW —
        // тоді між PA5 (анод D82) і PB6/PA6 (катод D82) ніколи не виникає напруга!
        pinMode(aPin, INPUT_PULLDOWN);
        digitalWrite(aPin, LOW);
        pinMode(cPin, (cPin == PA5 || cPin == PA6) ? INPUT_PULLDOWN : INPUT);
    } else {
        digitalWrite(cPin, LOW);
        pinMode(cPin, OUTPUT);
        digitalWrite(aPin, HIGH);
        pinMode(aPin, OUTPUT);

        delayMicroseconds(us);

        digitalWrite(aPin, LOW);
        pinMode(aPin, (aPin == PA5 || aPin == PA6) ? INPUT_PULLDOWN : INPUT);
        pinMode(cPin, (cPin == PA5 || cPin == PA6) ? INPUT_PULLDOWN : INPUT);
    }
}

// ------------------------------------------------------------------------------
// 2. МАТЕМАТИКА ТА ФІЗИЧНЕ ЯДРО СИМУЛЯЦІЇ РІДИНИ (Position-Based Fluids)
// ------------------------------------------------------------------------------
struct Vec2 {
    float x, y;
    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    constexpr Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    constexpr Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    constexpr Vec2 operator/(float s) const { return Vec2(x / s, y / s); }

    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(float s) { x *= s; y *= s; return *this; }

    constexpr float length_sq() const { return x * x + y * y; }
    float length() const { return sqrtf(length_sq()); }
};

struct FluidConfig {
    static constexpr uint16_t MAX_PARTICLES = 64;
    static constexpr uint16_t NOMINAL_PARTICLES = 28; // 60% заповнення (як у CLI)
    static constexpr float PARTICLE_RADIUS = 0.12f;
    static constexpr float DAMPING = 0.988f;
    static constexpr float REPULSION_STRENGTH = 0.92f;
    static constexpr uint16_t PHYSICAL_LEDS = 216; // 10+12+14 + 9*16 + 14+12+10 = 216 LED!
};

struct Particle {
    Vec2 pos;
    Vec2 vel;
};

struct HardwareLed {
    Vec2 pos;          // Нормалізована фізична координата [-1.0, 1.0]
    uint8_t anodePin;  // Пін анода STM32
    uint8_t cathodePin;// Пін катода STM32
    uint8_t pulseUs;   // Тривалість імпульсу свічення (0 = вимкнено, 4..18 мкс)
};

class EmbeddedFluidSimulator {
public:
    void init() {
        led_count_ = 0;
        for (int y = 0; y < 15; ++y) {
            for (int x = rowStart[y]; x <= rowEnd[y]; ++x) {
                if (led_count_ < FluidConfig::PHYSICAL_LEDS) {
                    // Центруємо сітку 16x15 у колі радіуса 1.0 (включно з кутами R15 D228..D237)
                    float nx = (static_cast<float>(x) - 7.5f) / 8.5f;
                    float ny = (static_cast<float>(y) - 7.0f) / 8.2f;
                    int aPin, cPin;
                    getLedPins(x, y, aPin, cPin);

                    leds_[led_count_].pos = Vec2(nx, ny);
                    leds_[led_count_].anodePin = static_cast<uint8_t>(aPin);
                    leds_[led_count_].cathodePin = static_cast<uint8_t>(cPin);
                    leds_[led_count_].pulseUs = 0;
                    ++led_count_;
                }
            }
        }
        resetParticles(FluidConfig::NOMINAL_PARTICLES);
    }

    void resetParticles(uint16_t target_count) {
        particle_count_ = 0;
        const float spacing = 0.135f;
        for (float y = 0.92f; y >= -0.92f && particle_count_ < target_count; y -= spacing) {
            for (int k = 0; k <= 14 && particle_count_ < target_count; ++k) {
                float x = (k == 0) ? 0.0f : ((k & 1) ? -((k + 1) / 2) * spacing : (k / 2) * spacing);
                if (x * x + y * y <= 0.92f) {
                    particles_[particle_count_].pos = Vec2(x, y);
                    particles_[particle_count_].vel = Vec2(0.0f, 0.0f);
                    ++particle_count_;
                }
            }
        }
        for (int i = 0; i < 20; ++i) {
            step(Vec2(0.0f, 9.8f), 0.025f, 0.0f, false);
        }
        for (uint16_t i = 0; i < particle_count_; ++i) {
            particles_[i].vel = Vec2(0.0f, 0.0f);
        }
        renderToLeds();
    }

    void adjustParticleCount(int delta) {
        if (delta > 0) {
            const float spacing = FluidConfig::PARTICLE_RADIUS * 1.15f;
            for (float y = 0.2f; y >= -0.8f && delta > 0 && particle_count_ < FluidConfig::MAX_PARTICLES; y -= spacing) {
                for (float x = -0.7f; x <= 0.7f && delta > 0 && particle_count_ < FluidConfig::MAX_PARTICLES; x += spacing) {
                    if (x * x + y * y <= 0.75f) {
                        particles_[particle_count_].pos = Vec2(x, y);
                        particles_[particle_count_].vel = Vec2(0.0f, 0.0f);
                        ++particle_count_;
                        --delta;
                    }
                }
            }
        } else if (delta < 0) {
            int rem = -delta;
            if (particle_count_ > static_cast<uint16_t>(rem + 5)) {
                particle_count_ -= rem;
            }
        }
        renderToLeds();
    }

    void addImpulse(Vec2 impulse) {
        for (uint16_t i = 0; i < particle_count_; ++i) {
            particles_[i].vel += impulse;
        }
    }

    void addRotationalImpulse(float omega) {
        if (fabsf(omega) < 1e-4f) return;
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

    // Один швидкий прохід POV-матриці (~1.1 мс) — викликається і в циклі, і між підкроками фізики
    inline void scanOnce() {
        for (uint16_t k = 0; k < led_count_; ++k) {
            uint8_t us = leds_[k].pulseUs;
            if (us == 0) continue;
            pulseLedSafe(leds_[k].anodePin, leds_[k].cathodePin, us);
        }
    }

    void step(Vec2 gravity, float dt, float angular_velocity = 0.0f, bool interleaveScan = true) {
        if (dt <= 0.0f) return;
        if (dt > 0.033f) dt = 0.033f;

        Vec2 old_pos[FluidConfig::MAX_PARTICLES];

        // 1. Передбачення позицій під дією гравітації та обертання кулона
        for (uint16_t i = 0; i < particle_count_; ++i) {
            old_pos[i] = particles_[i].pos;
            particles_[i].vel *= FluidConfig::DAMPING;
            particles_[i].vel += gravity * dt;

            if (fabsf(angular_velocity) > 0.01f) {
                float r = particles_[i].pos.length();
                if (r > 0.25f) {
                    Vec2 v_wall(angular_velocity * particles_[i].pos.y,
                               -angular_velocity * particles_[i].pos.x);
                    float coupling = ((r - 0.25f) / 0.72f) * 0.35f;
                    if (coupling > 0.40f) coupling = 0.40f;
                    particles_[i].vel += (v_wall - particles_[i].vel) * coupling;
                }
            }
            particles_[i].pos += particles_[i].vel * dt;
        }

        // 2. Ітеративне розв'язання геометричних обмежень нестисливості (PBF)
        for (int iter = 0; iter < 4; ++iter) {
            resolveCollisions();
            applyBoundary();
        }

        // Підтримуємо свічення матриці всередині фізичного кроку (нуль пауз = нуль мерехтіння!)
        if (interleaveScan) scanOnce();

        // 3. Оновлення швидкості з реального переміщення: v = (x_new - x_old) / dt
        float inv_dt = 1.0f / dt;
        for (uint16_t i = 0; i < particle_count_; ++i) {
            particles_[i].vel = (particles_[i].pos - old_pos[i]) * inv_dt;
            particles_[i].vel *= FluidConfig::DAMPING;
        }

        // 4. XSPH в'язкість (когезія поверхні та згладжування бризок)
        const float visc_radius = FluidConfig::PARTICLE_RADIUS * 2.2f;
        const float visc_radius_sq = visc_radius * visc_radius;
        const float inv_visc_sq = 1.0f / visc_radius_sq;
        const float c_visc = 0.015f;
        for (uint16_t i = 0; i < particle_count_; ++i) {
            for (uint16_t j = i + 1; j < particle_count_; ++j) {
                Vec2 delta = particles_[i].pos - particles_[j].pos;
                float dist_sq = delta.length_sq();
                if (dist_sq < visc_radius_sq && dist_sq > 1e-6f) {
                    float q = 1.0f - (dist_sq * inv_visc_sq);
                    Vec2 visc_impulse = (particles_[j].vel - particles_[i].vel) * (c_visc * q);
                    particles_[i].vel += visc_impulse;
                    particles_[j].vel -= visc_impulse;
                }
            }
        }

        // 5. Швидка растеризація частинок на 216 фізичних світлодіодів (без sqrtf!)
        renderToLeds();
        if (interleaveScan) scanOnce();
    }

    // Високочастотне POV-відтворення поточного кадру (> 800 Гц, без мерехтіння)
    void displayFrame(uint32_t frame_ms) {
        uint32_t t0 = millis();
        while (millis() - t0 < frame_ms) {
            scanOnce();
        }
    }

private:
    void resolveCollisions() {
        const float min_dist = FluidConfig::PARTICLE_RADIUS * 2.0f;
        const float min_dist_sq = min_dist * min_dist;

        for (uint16_t i = 0; i < particle_count_; ++i) {
            for (uint16_t j = i + 1; j < particle_count_; ++j) {
                Vec2 delta = particles_[i].pos - particles_[j].pos;
                float dist_sq = delta.length_sq();
                if (dist_sq < min_dist_sq && dist_sq > 1e-6f) {
                    float dist = sqrtf(dist_sq);
                    float overlap = min_dist - dist;
                    Vec2 push = (delta / dist) * (0.5f * overlap * FluidConfig::REPULSION_STRENGTH);
                    particles_[i].pos += push;
                    particles_[j].pos -= push;
                }
            }
        }
    }

    void applyBoundary() {
        const float max_radius = 0.96f;
        for (uint16_t i = 0; i < particle_count_; ++i) {
            float dist_sq = particles_[i].pos.length_sq();
            if (dist_sq > max_radius * max_radius) {
                float dist = sqrtf(dist_sq);
                particles_[i].pos *= (max_radius / dist);
            }
        }
    }

    // Надшвидка растеризація через поліноміальне ядро (1 - d^2 / R^2)^2 без жодного sqrtf()!
    void renderToLeds() {
        const float influence_radius = FluidConfig::PARTICLE_RADIUS * 2.05f;
        const float influence_sq = influence_radius * influence_radius;
        const float inv_influence_sq = 1.0f / influence_sq;

        for (uint16_t k = 0; k < led_count_; ++k) {
            float density = 0.0f;
            const float lx = leds_[k].pos.x;
            const float ly = leds_[k].pos.y;

            for (uint16_t i = 0; i < particle_count_; ++i) {
                float dx = lx - particles_[i].pos.x;
                float dy = ly - particles_[i].pos.y;
                float d2 = dx * dx + dy * dy;
                if (d2 < influence_sq) {
                    float q = 1.0f - d2 * inv_influence_sq;
                    density += q * q * q;
                }
            }

            // Короткі високочастотні імпульси (0..18 мкс): у 3 рази вища частота сканування без мерехтіння!
            if (density <= 0.03f) {
                leds_[k].pulseUs = 0;
            } else if (density < 0.22f) {
                leds_[k].pulseUs = 5;  // Тонка кромка хвилі / бризки
            } else if (density < 0.50f) {
                leds_[k].pulseUs = 11; // Середня глибина
            } else {
                leds_[k].pulseUs = 18; // Яскраве ядро рідини
            }
        }
    }

    Particle particles_[FluidConfig::MAX_PARTICLES];
    uint16_t particle_count_ = 0;
    HardwareLed leds_[FluidConfig::PHYSICAL_LEDS];
    uint16_t led_count_ = 0;
};

// ------------------------------------------------------------------------------
// 3. БЕЗКОНФЛІКТНИЙ OPEN-DRAIN ДРАЙВЕР MPU6050 НА D5 (PB6) ТА D4 (PB7)
// ------------------------------------------------------------------------------
namespace ImuBus {
    int pinSda = PB7;     // D4
    int pinSdaPair = PA5; // парний пін через SB18
    int pinScl = PB6;     // D5
    int pinSclPair = PA6; // парний пін через SB16

    void setPins(int scl, int sda) {
        pinScl = scl;
        pinSclPair = (scl == PB6) ? PA6 : PA5;
        pinSda = sda;
        pinSdaPair = (sda == PB7) ? PA5 : PA6;

        ImuPins::sclMcu  = static_cast<uint8_t>(pinScl);
        ImuPins::sclLine = static_cast<uint8_t>(pinSclPair);
        ImuPins::sdaMcu  = static_cast<uint8_t>(pinSda);
        ImuPins::sdaLine = static_cast<uint8_t>(pinSdaPair);
    }

    // Усі 14 не-I2C ліній матриці залишаються в INPUT (Hi-Z), щоб струм через діагоналі = 0!
    inline void beginBus() {
        pinMode(PA5, INPUT_PULLUP);
        pinMode(PA6, INPUT_PULLUP);
        pinMode(PB6, INPUT_PULLUP);
        pinMode(PB7, INPUT_PULLUP);
    }

    // Чистий Open-Drain: НІКОЛИ не вмикаємо OUTPUT HIGH між D5 і D4 (щоб D81/D82 не світилися!)
    inline void sdaHigh() {
        pinMode(pinSdaPair, INPUT_PULLUP);
        pinMode(pinSda, INPUT_PULLUP);
    }
    inline void sdaLow() {
        pinMode(pinSdaPair, INPUT);
        pinMode(pinSda, OUTPUT);
        digitalWrite(pinSda, LOW);
    }
    inline void sclHigh() {
        pinMode(pinSclPair, INPUT_PULLUP);
        pinMode(pinScl, INPUT_PULLUP);
    }
    inline void sclLow() {
        pinMode(pinSclPair, INPUT);
        pinMode(pinScl, OUTPUT);
        digitalWrite(pinScl, LOW);
    }
    inline void i2cDelay() { delayMicroseconds(1); }

    // Якщо MPU6050 випадково утримує SDA в LOW — звільняємо шину 9 тактами SCL + STOP
    void recoverBusIfStuck() {
        sdaHigh();
        sclHigh();
        delayMicroseconds(2);
        if (digitalRead(pinSda) == LOW) {
            for (int i = 0; i < 9; ++i) {
                sclLow();  i2cDelay();
                sclHigh(); i2cDelay();
                if (digitalRead(pinSda) == HIGH) break;
            }
        }
    }

    void start() {
        sdaHigh(); sclHigh(); i2cDelay();
        sdaLow();  i2cDelay();
        sclLow();  i2cDelay();
    }

    void stop() {
        sdaLow();  i2cDelay();
        sclHigh(); i2cDelay();
        sdaHigh(); i2cDelay();
    }

    bool writeByte(uint8_t b) {
        for (int i = 7; i >= 0; --i) {
            if ((b >> i) & 1) sdaHigh(); else sdaLow();
            i2cDelay();
            sclHigh(); i2cDelay();
            sclLow();  i2cDelay();
        }
        sdaHigh(); i2cDelay();
        sclHigh(); i2cDelay();
        bool ack = (digitalRead(pinSda) == LOW);
        sclLow();  i2cDelay();
        return ack;
    }

    uint8_t readByte(bool sendAck) {
        uint8_t b = 0;
        sdaHigh();
        for (int i = 7; i >= 0; --i) {
            i2cDelay();
            sclHigh(); i2cDelay();
            if (digitalRead(pinSda)) b |= (1 << i);
            sclLow();
        }
        if (sendAck) sdaLow(); else sdaHigh();
        i2cDelay();
        sclHigh(); i2cDelay();
        sclLow();  i2cDelay();
        sdaHigh();
        return b;
    }

    bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
        beginBus();
        recoverBusIfStuck();
        start();
        if (!writeByte(addr << 1)) { stop(); clearAll(); return false; }
        if (!writeByte(reg))       { stop(); clearAll(); return false; }
        if (!writeByte(val))       { stop(); clearAll(); return false; }
        stop();
        clearAll();
        return true;
    }

    bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
        beginBus();
        recoverBusIfStuck();
        start();
        if (!writeByte(addr << 1)) { stop(); clearAll(); return false; }
        if (!writeByte(reg))       { stop(); clearAll(); return false; }
        start();
        if (!writeByte((addr << 1) | 1)) { stop(); clearAll(); return false; }
        for (uint8_t i = 0; i < len; ++i) {
            buf[i] = readByte(i + 1 < len);
        }
        stop();
        clearAll();
        return true;
    }
}

enum ImuType { IMU_NONE = 0, IMU_MPU6050, IMU_LSM6DS3 };
ImuType activeImu = IMU_NONE;
uint8_t imuAddr = 0;

// Компенсація кута припаювання MPU6050 на платі (45 градусів = PI/4 рад за замовчуванням)
float imuAngleRad = 0.78539816f;
float lastRawAx = 0.70710678f;
float lastRawAy = 0.70710678f;

void calibrateImuDown() {
    if (lastRawAx * lastRawAx + lastRawAy * lastRawAy > 0.09f) {
        // Повертаємо систему координат так, щоб поточний вектор (lastRawAx, lastRawAy) дивився строго вниз (0, +g)
        imuAngleRad = atan2f(lastRawAx, lastRawAy);
    }
}

void detectAndInitImu() {
    const int sclCandidates[2] = {PB6, PB7};
    const int sdaCandidates[2] = {PB7, PB6};
    const uint8_t mpuAddrs[2]  = {0x68, 0x69};

    for (int p = 0; p < 2; ++p) {
        ImuBus::setPins(sclCandidates[p], sdaCandidates[p]);

        for (uint8_t a : mpuAddrs) {
            if (ImuBus::writeReg(a, 0x6B, 0x00)) { // PWR_MGMT_1 = 0x00 (вихід зі сну)
                ImuBus::writeReg(a, 0x19, 0x07);   // SMPLRT_DIV = 7
                ImuBus::writeReg(a, 0x1A, 0x03);   // CONFIG: DLPF 44 Hz
                ImuBus::writeReg(a, 0x1B, 0x00);   // GYRO_CONFIG: +/-250 dps
                ImuBus::writeReg(a, 0x1C, 0x00);   // ACCEL_CONFIG: +/-2g
                activeImu = IMU_MPU6050;
                imuAddr = a;
                Serial.print("MPU6050 OK addr=0x");
                Serial.println(a, HEX);
                return;
            }
        }
    }
    ImuBus::setPins(PB6, PB7);
}

// Швидке зчитування гравітації з MPU6050 (лише 4 байти ACCEL_X/Y раз на 40 мс = ~100 мкс!)
bool readImuMotion(Vec2 &gravity, float &omega_z, Vec2 &jerkImpulse) {
    if (activeImu == IMU_NONE) return false;

    static uint32_t lastReadMs = 0;
    static Vec2 filtered_g(0.0f, 9.81f);
    static bool hasValidReading = false;

    jerkImpulse = Vec2(0.0f, 0.0f);
    omega_z = 0.0f;

    uint32_t now = millis();
    if (now - lastReadMs < 40) {
        gravity = filtered_g;
        return hasValidReading;
    }
    lastReadMs = now;

    uint8_t buf[4];
    if (activeImu == IMU_MPU6050) {
        // Зчитуємо лише 4 байти ACCEL_XOUT_H..ACCEL_YOUT_L (0x3B..0x3E) за ~100 мкс!
        if (ImuBus::readRegs(imuAddr, 0x3B, buf, 4)) {
            int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);

            if (!((raw_ax == -1 && raw_ay == -1) || (raw_ax == 0 && raw_ay == 0))) {
                float ax_g = raw_ax / 16384.0f;
                float ay_g = raw_ay / 16384.0f;
                lastRawAx = ax_g;
                lastRawAy = ay_g;

                // Повертаємо вектор гравітації на кут припаювання MPU6050 (за замовчуванням 45 градусів):
                float c = cosf(imuAngleRad);
                float s = sinf(imuAngleRad);
                float rot_ax = ax_g * c - ay_g * s;
                float rot_ay = ax_g * s + ay_g * c;

                Vec2 raw_g(rot_ax * 9.81f, rot_ay * 9.81f);

                // Виявлення різкого струшування (shake impulse)
                Vec2 dg = raw_g - filtered_g;
                if (hasValidReading && dg.length_sq() > 22.0f) {
                    jerkImpulse = dg * 0.15f;
                }

                // Плавне згладжування вектора гравітації між кадрами
                if (!hasValidReading) {
                    filtered_g = raw_g;
                    hasValidReading = true;
                } else {
                    filtered_g = filtered_g * 0.35f + raw_g * 0.65f;
                }
            }
        }
    }

    gravity = filtered_g;
    return hasValidReading;
}

// ------------------------------------------------------------------------------
// 4. ГОЛОВНИЙ ЦИКЛ ПРОШИВКИ (40 FPS СИМУЛЯЦІЯ + КЕРУВАННЯ З CLI / IMU)
// ------------------------------------------------------------------------------
EmbeddedFluidSimulator fluidSim;
bool autoRotate = false;
float autoAngle = 0.0f;
float rotateSpeed = 0.12f;
Vec2 dynamicAcc(0.0f, 0.0f);

void handleSerialCommand(char c) {
    switch (c) {
        case ' ':
            autoRotate = !autoRotate;
            break;
        case 'f': case 'F':
            if (rotateSpeed < 0.35f) rotateSpeed += 0.03f;
            break;
        case 'v': case 'V':
            if (rotateSpeed > 0.03f) rotateSpeed -= 0.03f;
            break;
        case 'e': case 'E':
            autoRotate = false;
            fluidSim.addRotationalImpulse(6.0f);
            break;
        case 'r': case 'R':
            autoRotate = false;
            fluidSim.addRotationalImpulse(-6.0f);
            break;
        case 'a': case 'A':
            autoRotate = false;
            fluidSim.addImpulse(Vec2(-4.2f, -1.0f));
            dynamicAcc.x -= 32.0f;
            break;
        case 'd': case 'D':
            autoRotate = false;
            fluidSim.addImpulse(Vec2(4.2f, -1.0f));
            dynamicAcc.x += 32.0f;
            break;
        case 'w': case 'W':
            autoRotate = false;
            fluidSim.addImpulse(Vec2(0.0f, -5.5f));
            dynamicAcc.y -= 35.0f;
            break;
        case 's': case 'S':
            autoRotate = false;
            fluidSim.addImpulse(Vec2(0.0f, 4.0f));
            dynamicAcc.y += 24.0f;
            break;
        case 'x': case 'X': {
            autoRotate = false;
            float rx = ((millis() % 40) - 20) * 0.25f;
            float ry = (((millis() >> 3) % 40) - 20) * 0.25f;
            fluidSim.addImpulse(Vec2(rx, ry));
            dynamicAcc.x += rx * 10.0f;
            dynamicAcc.y += ry * 10.0f;
            break;
        }
        case '+': case '=':
            fluidSim.adjustParticleCount(8);
            break;
        case '-': case '_':
            fluidSim.adjustParticleCount(-8);
            break;
        case '1':
            fluidSim.resetParticles(8);
            break;
        case '2':
            fluidSim.resetParticles(18);
            break;
        case '3':
            fluidSim.resetParticles(28);
            break;
        case '4':
            fluidSim.resetParticles(40);
            break;
        case '5':
            fluidSim.resetParticles(52);
            break;
        case 'c': case 'C':
            autoRotate = false;
            dynamicAcc = Vec2(0.0f, 0.0f);
            calibrateImuDown();
            fluidSim.resetParticles(28);
            break;
        default:
            break;
    }
}

void setup() {
    // Оскільки L[0] = PB3, піни UART (PA2/PA15) повністю вільні!
    Serial.begin(115200);
    pinMode(PA12, INPUT);
    clearAll();
    delay(60); // Час на стабілізацію живлення MPU6050
    fluidSim.init();
    detectAndInitImu();
}

void loop() {
    // 1. Зчитуємо всі команди з клавіатури (з ./build/fluid_sim_cli або Serial Monitor)
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        handleSerialCommand(c);
    }

    Vec2 total_g(0.0f, 9.8f);
    float current_omega = 0.0f;
    Vec2 shakeImpulse(0.0f, 0.0f);

    // 2. Опитування MPU6050 (раз на 40 мс, триває лише ~100 мкс у чистому Open-Drain)
    if (readImuMotion(total_g, current_omega, shakeImpulse)) {
        if (shakeImpulse.length_sq() > 0.01f) {
            fluidSim.addImpulse(shakeImpulse);
        }
        total_g.x += dynamicAcc.x;
        total_g.y += dynamicAcc.y;
    } else if (autoRotate) {
        autoAngle += rotateSpeed;
        current_omega = rotateSpeed / 0.025f;
        total_g = Vec2(sinf(autoAngle) * 9.8f, cosf(autoAngle) * 9.8f);
    } else {
        total_g = Vec2(dynamicAcc.x, 9.8f + dynamicAcc.y);
    }

    dynamicAcc.x *= 0.93f;
    dynamicAcc.y *= 0.93f;
    if (fabsf(dynamicAcc.x) < 0.2f) dynamicAcc.x = 0.0f;
    if (fabsf(dynamicAcc.y) < 0.2f) dynamicAcc.y = 0.0f;

    // 3. Крок фізики Position-Based Fluids із вбудованим POV-скануванням між етапами
    fluidSim.step(total_g, 0.025f, current_omega, true);

    // 4. Високочастотне POV-відображення кадру (> 800 Гц, без мерехтіння)
    fluidSim.displayFrame(22);
}

