# CLAUDE.md

## Роль этого файла

Этот файл является технической спецификацией для Claude как для implementer-а. Его задача: реализовать веб-приложение симуляции роя дронов без GPS так, чтобы Codex мог валидировать его как инженерно правдоподобную модель, а не как визуальный mock.

Если между простотой реализации и корректностью модели есть конфликт, приоритет у корректности модели. Упрощения допустимы только если они:

- явно задокументированы;
- параметризованы;
- локализованы;
- не искажают ключевые выводы симуляции;
- не подменяют физику прямым управлением состоянием.

## 1. Цель проекта

Нужен браузерный симулятор swarm flight для мультикоптеров, где:

- нет GPS в штатном контуре;
- полет идет в полном 3D;
- у каждого агента есть real dynamics, sensors, estimator, controller и communication stack;
- можно моделировать помехи, деградации и отказы;
- можно тестировать formation control, cooperative navigation, relative localization и collision avoidance.

Это не игра и не motion-design сцена. Это research-grade simulation framework в браузере.

При этом интерфейс не должен быть спартанским или "только для разработчика". Нужен современный, понятный и responsive UI, через который можно управлять почти всеми значимыми параметрами симуляции, сценариями, шумами, деградациями и режимами визуализации.

## 2. Архитектурная декомпозиция

Проект должен быть разбит на 6 слоев.

### 2.1 Truth Layer

Хранит истинное состояние мира:

- позы, скорости, угловые скорости дронов;
- реальные моторные скорости;
- истинное поле ветра;
- истинное магнитное поле;
- истинное давление/температуру;
- геометрию мира;
- истинные положения препятствий и других агентов.

### 2.2 Sensor Layer

Генерирует измерения из truth state с учетом:

- noise;
- bias;
- random walk;
- latency;
- sampling rate;
- jitter;
- clipping;
- dropout;
- outlier;
- aliasing;
- калибровочных ошибок.

### 2.3 Estimation Layer

Строит оценку состояния только по noisy measurements.

Должны существовать минимум два режима:

1. fast engineering mode:
   error-state EKF / MEKF / IEKF;
2. high-fidelity mode:
   VIO/SLAM/UWB fusion с IMU preintegration и/или factor graph.

### 2.4 Guidance and Swarm Layer

Формирует желаемое поведение:

- следование траектории;
- formation keeping;
- relative positioning;
- consensus;
- cooperative target pursuit;
- reconfiguration after failures.

### 2.5 Safety Layer

Отвечает за жесткие ограничения:

- collision avoidance;
- separation minima;
- no-fly zones;
- speed and tilt bounds;
- failsafe behavior.

Safety layer не должна быть только частью formation controller. Это отдельный слой, который может переопределить команду.

### 2.6 Presentation Layer

Отвечает за:

- 3D rendering;
- logs;
- plots;
- overlays;
- replay;
- scenario configuration.

UI не должен вмешиваться в physics timing.

Presentation layer обязан быть:

- современным по UX;
- понятным для длительной исследовательской работы;
- responsive на desktop и узких экранах;
- пригодным для тонкой настройки параметров без редактирования кода.

Минимально в UI должны быть:

- глобальная панель запуска, паузы, reset, replay, step;
- scene/scenario selector;
- инспектор выбранного дрона;
- панели параметров physics, sensors, estimation, communication, swarm, safety;
- редактор шумов и деградаций;
- отображение truth vs estimate vs command;
- графики ключевых сигналов и метрик;
- визуальные маркеры `validated`, `simplified`, `stub`.

## 3. Системы координат

Нужно явно определить и поддерживать:

- `W`: world / inertial frame;
- `B`: body frame drone;
- `S_k`: frame конкретного сенсора;
- `C`: camera frame;
- `U`: UWB/radio reference conventions if needed.

Рекомендуемый базовый выбор:

- world frame: ENU или NED, но единообразно во всем проекте;
- body frame: `x` forward, `y` right, `z` down или `z` up, но строго согласованно с матрицами и mixer.

Нужно завести единый документ `frames.ts` или аналог, где явно описано:

- направление осей;
- sign convention;
- rotation order;
- quaternion convention;
- mapping between frames.

Главная ошибка, которую нельзя допустить: смешение ENU/NED и body-up/body-down знаков в разных подсистемах.

## 4. Состояние одного дрона

Минимальный truth state одного мультикоптера:

`x_truth = { p_w, v_w, q_wb or R_wb, omega_b, Omega_1..Omega_4, bg, ba, bm, bbaro, motor_states, thermal_states(optional), health_states }`

Где:

- `p_w in R^3`: позиция;
- `v_w in R^3`: линейная скорость;
- `R_wb in SO(3)` или `q_wb`: ориентация body относительно world;
- `omega_b in R^3`: угловая скорость в body frame;
- `Omega_i`: угловые скорости роторов;
- `bg, ba`: gyro/acc biases;
- `bm`: magnetometer bias/disturbance proxy;
- `bbaro`: baro bias/drift state.

Для estimator state не обязан совпадать с truth state, но должен быть достаточно богат:

`x_est ~= [p, v, q, bg, ba, bwind, bbaro, optional visual/UWB states]`

## 5. Базовая 6DoF динамика

Для каждого дрона должна быть реальная rigid-body model.

### 5.1 Трансляционная динамика

В мировой системе:

`dot(p) = v`

`m dot(v) = m g + R_wb F_b + F_aero_w + F_wind_w + F_contact_w + F_dist_w`

Где:

- `m` - масса;
- `g` - вектор гравитации;
- `F_b = [0, 0, -sum(T_i)]` или эквивалент согласно выбранной оси body;
- `F_aero_w` - аэродинамическое сопротивление;
- `F_wind_w` - силы из-за ветра и gusts;
- `F_contact_w` - реакции земли/стен при контакте, если контакт моделируется;
- `F_dist_w` - прочие внешние возмущения.

### 5.2 Ротационная динамика

`dot(R) = R hat(omega)`

`J dot(omega) = tau_total - omega x (J omega)`

или в quaternion form:

`dot(q) = 0.5 Omega(omega) q`

Где `tau_total` включает:

- thrust-generated torques from rotor geometry;
- rotor drag torques;
- aerodynamic moments;
- gyroscopic terms;
- disturbance torques.

### 5.3 Обязательные физические параметры

У дрона должны быть параметризованы:

- `m`;
- `J = diag(Jx, Jy, Jz)` или full inertia tensor;
- arm length `l`;
- rotor positions;
- rotor spin directions;
- thrust coefficient `k_T`;
- drag torque coefficient `k_Q`;
- motor lag constants;
- actuator saturation limits;
- maximum thrust / maximum rotor speed;
- body drag coefficients;
- projected areas;
- sensor extrinsics.

## 6. Модель двигателя, ESC и роторов

Нельзя подавать управляющее воздействие напрямую как мгновенную тягу.

Нужна хотя бы такая цепочка:

`u_cmd -> ESC/motor model -> rotor speed Omega_i -> thrust/torque`

### 6.1 Motor dynamics

Минимально:

`dot(Omega_i) = (Omega_cmd_i - Omega_i) / tau_m`

С обязательными эффектами:

- first-order lag;
- lower/upper saturation;
- rate limit;
- dead zone near zero;
- optional command quantization.

### 6.2 Rotor thrust and drag

Baseline:

`T_i = k_T Omega_i^2`

`Q_i = s_i k_Q Omega_i^2`

Где `s_i in {-1, +1}` зависит от направления вращения.

### 6.3 Mixer

Нужна матрица преобразования между:

- collective thrust / roll / pitch / yaw commands
- и индивидуальными роторами.

Mixer должен учитывать:

- конфигурацию `X` или `+`;
- saturation redistribution;
- partial actuator failure modes.

### 6.4 Что желательно добавить на high-fidelity уровне

- rotor drag in body plane;
- blade flapping proxy;
- thrust dependence on inflow velocity;
- battery voltage sag effect on maximum available thrust;
- motor heating proxy;
- partial loss of efficiency.

## 7. Аэродинамика и внешняя среда

Простейшая модель "без воздуха" недостаточна.

### 7.1 Относительная скорость относительно воздуха

`v_rel_w = v_w - w_w(p, t)`

`v_rel_b = R_bw v_rel_w`

Все аэродинамические силы должны строиться от `v_rel`, а не от ground speed.

### 7.2 Корпусное сопротивление

Минимально допустимы два варианта:

1. linear drag:
   `F_drag_b = -D v_rel_b`
2. quadratic drag:
   `F_drag_b = -0.5 rho C_D A |v_rel_b| v_rel_b`

Рекомендуется поддержать оба.

### 7.3 Rotor drag

Для более реалистичного агрессивного полета добавь модель linear rotor drag по мотивам Faessler et al.:

`F_rotor_drag_b ~= -D_r v_rel_b`

Это особенно важно, если симуляция должна правдоподобно вести себя в fast forward flight и при feed-forward tracking.

### 7.4 Ветер

Поле ветра должно быть пространственно и временно зависящим:

`w_w(p, t) = w_mean(p) + w_gust(p, t) + w_turb(p, t) + w_local(p, t)`

Минимальные компоненты:

- mean wind;
- gust events;
- turbulence;
- obstacle-induced local flow.

### 7.5 Turbulence

Для инженерного baseline достаточно Dryden-like colored noise field.

Нужны параметры:

- turbulence intensity;
- integral length scale;
- correlation time;
- spatial correlation between nearby drones.

Недопустимо моделировать turbulence только как iid white noise по осям.

### 7.6 Ground effect

При малых высотах тяга и поток искажаются. Нужен хотя бы приближенный множитель:

`T_eff = T_nom * k_ge(h / R_prop)`

Где `k_ge > 1` при работе близко к земле, но с адекватным ограничением.

Нужно учитывать:

- увеличение эффективной тяги;
- нестабильность near-ground hover;
- изменение шумов range/baro/optical-flow near ground.

### 7.7 Wall/Ceiling effect

Для indoor environments желательно заложить расширяемую модель:

- измененная циркуляция и rotorwash около стен/потолка;
- локальные аэродинамические импульсы;
- изменение optical flow и range readings.

Даже если сначала это упрощено, интерфейс должен позволять добавить позже.

## 8. Атмосфера и геофизика

### 8.1 Барометрическая атмосфера

Используй стандартную атмосферу ISA/US Standard Atmosphere как baseline:

- pressure as function of altitude;
- density as function of altitude;
- temperature lapse rate.

Для малых indoor/outdoor высот достаточно troposphere model.

### 8.2 Барометр

Нужно моделировать не только white noise, но и:

- bias;
- random walk;
- temperature drift;
- static pressure position error при горизонтальном движении;
- low-pass dynamics;
- quantization.

### 8.3 Магнитное поле

Earth magnetic field нельзя брать как константу `[1,0,0]`.

Нужно:

- baseline field из WMM/WMMHR for chosen location/date;
- inclination and declination;
- local magnetic anomalies;
- onboard electromagnetic disturbances from motors/ESC/power lines;
- hard-iron and soft-iron distortion.

Рекомендуемая логика:

`B_meas = M_soft * ( R_bw B_earth_w + B_local_b ) + b_hard + n`

Где `B_local_b` зависит от тока, throttle и окружения.

## 9. Сенсоры

### 9.1 IMU

IMU обязана быть high-rate и multi-error.

Базовая модель:

`omega_meas = omega_true + bg + ng + Sg omega_true + Mg omega_true + vib_g`

`a_meas = R_bw ( dot(v) - g ) + ba + na + Sa a_true + Ma a_true + vib_a`

Где:

- `bg, ba` - biases;
- `ng, na` - white noise;
- `S*` - scale factor errors;
- `M*` - axis misalignment / cross-axis coupling;
- `vib_*` - vibration/aliasing term.

Bias dynamics:

`dot(bg) = w_bg`

`dot(ba) = w_ba`

Минимально нужны:

- white noise density;
- bias random walk;
- saturation;
- finite sampling;
- timestamp jitter;
- optional clipping.

Очень желательно:

- Allan-variance-derived parameter sets;
- vibration aliasing mode;
- temperature-dependent bias drift.

### 9.2 Magnetometer

Нужно моделировать:

- WMM baseline field;
- hard iron;
- soft iron;
- electromagnetic interference;
- saturation;
- dropout / innovation rejection cases.

Модель:

`m_meas = M_soft ( R_bw B_earth_w + B_env_b + B_motor_b ) + b_hard + n_m`

`B_motor_b` можно аппроксимировать как функция тока/rotor speed.

### 9.3 Barometer

Измерение:

`p_meas = p_atm(h) + b_baro + d_temp + d_static(v_rel, attitude) + n_baro`

Высота затем вычисляется через обратную barometric formula.

Нужно поддержать:

- correlated noise;
- slow ambient drift;
- static pressure error from airspeed and orientation.

### 9.4 Rangefinder / LiDAR

Модель должна учитывать:

- beam direction;
- min/max range;
- incidence angle;
- invalid returns;
- multipath/reflection probability;
- occlusion by drone body/payload;
- update rate and latency.

Измерение нельзя делать просто `z = distance_to_ground`.

### 9.5 Optical Flow

Если используется optical flow, обязателен downward sensor + distance dependence.

Нужно моделировать:

- зависимость точности от высоты;
- dependence on surface texture;
- motion blur;
- rolling shutter optional;
- lighting changes;
- feature starvation;
- low-quality metric;
- integration over exposure window;
- outliers over reflective or repetitive patterns.

Важная связь:

optical-flow velocity scale зависит от distance sensor.

### 9.6 Camera / VIO sensor

Для VIO/VSLAM нужны:

- intrinsics;
- extrinsics camera-IMU;
- lens distortion;
- frame rate;
- exposure;
- latency;
- rolling shutter optional;
- blur;
- motion blur;
- photometric variation;
- dynamic occluders;
- feature-poor scenes.

Если сцена слишком бедна текстурой, estimator должен реально деградировать.

### 9.7 UWB

UWB нужен как отдельная опция для relative or global indoor aiding.

Модель должна поддерживать:

- inter-drone ranging;
- anchor-based ranging;
- LOS vs NLOS mode;
- positive bias under NLOS;
- packet loss;
- asynchronous measurements;
- MAC/channel occupancy effects at large swarm sizes.

Базовая модель range:

`r_meas = ||p_i - p_j|| + b_LOS/NLOS + n_r`

При NLOS bias не должен быть симметричным Gaussian around zero. Он чаще положительный и может быть тяжелохвостым.

### 9.8 Communications

Связь между агентами и с инфраструктурой должна моделироваться отдельно от UWB range sensor.

Нужно поддержать:

- latency;
- jitter;
- drop rate;
- out-of-order delivery optional;
- finite bandwidth;
- neighbor graph dynamics.

Нельзя предполагать идеальный all-to-all communication.

## 10. Математика оценивания состояния

### 10.1 Почему estimator обязателен

В GPS-denied mode любая правдоподобная архитектура живет через estimator. Управление по truth state недопустимо.

### 10.2 Минимальный estimator

Для baseline можно начать с error-state EKF / MEKF:

состояние:

`x = [p, v, q, bg, ba, bbaro, bwind(optional)]`

propagation:

- position from velocity;
- velocity from specific force;
- attitude from gyro integration;
- bias random walk.

updates:

- magnetometer for heading;
- barometer for altitude;
- range for local height;
- optical flow for planar velocity;
- UWB for range constraints;
- vision for pose/feature updates.

### 10.3 Error-state representation

Рекомендуемый error vector:

`delta x = [delta p, delta v, delta theta, delta bg, delta ba, ...]`

Attitude error должен жить в малом угле, а не как прямое вычитание quaternion.

### 10.4 Важные вопросы наблюдаемости

Claude обязан учитывать:

- monocular vision без IMU не дает metric scale;
- heading с магнитометром нестабилен при EMI;
- без магнетометра и без визуальных глобальных ориентиров yaw drift неизбежен;
- altitude по baro и range может расходиться near ground / in airflow;
- UWB ranges alone не всегда дают наблюдаемость orientation;
- relative localization не равна абсолютной локализации.

### 10.5 High-fidelity estimation

Для продвинутого режима стоит заложить возможность:

- IMU preintegration;
- factor graph backend;
- visual reprojection factors;
- UWB range factors;
- loop closure optional;
- map reuse optional.

Это нужно не обязательно сразу реализовать полностью, но архитектурно нельзя закрывать путь к этому.

## 11. Управление одним дроном

Нужен каскадный стек.

### 11.1 Guidance

Генерирует desired:

- position / velocity / acceleration;
- yaw / heading;
- or full SE(3) reference.

### 11.2 Outer loop

Использует estimated state и вычисляет:

- desired thrust direction;
- desired collective thrust;
- desired attitude.

### 11.3 Inner loop

Отвечает за:

- attitude tracking;
- rate tracking;
- motor commands.

### 11.4 Рекомендуемые контроллеры

Допустимые варианты:

- geometric SE(3) controller;
- cascaded nonlinear controller;
- PID/LQR as baseline only if dynamics respected.

Предпочтительно:

- geometric controller for attitude and trajectory tracking;
- feed-forward terms for aggressive trajectories;
- optional drag-aware feed-forward.

## 12. Swarm layer

### 12.1 Что не надо делать

Не строить всю swarm logic только на классических boids правилах:

- separation;
- alignment;
- cohesion.

Boids можно использовать как visual heuristic или low-priority behavior prior, но не как единственный flight-control law.

### 12.2 Что нужно делать

Swarm architecture должна содержать:

- neighbor graph;
- relative localization;
- communication model;
- formation representation;
- collision avoidance;
- reconfiguration logic.

### 12.3 Формирование строя

Нужно поддержать хотя бы два режима:

1. leader-follower;
2. distributed formation / consensus / relative-position based.

Опционально:

- virtual structure;
- bearing-only formation;
- target pursuit.

### 12.4 Relative localization

Для GPS-denied swarm относительное позиционирование - центральная часть системы.

Нужно поддержать комбинации:

- UWB relative ranging;
- monocular/stereo relative vision;
- shared VIO map or inter-agent observations;
- communication-delivered neighbor estimates.

### 12.5 Collision avoidance

Должен быть hard-safety слой. Предпочтительные подходы:

- ORCA/RVO-style local collision avoidance;
- control barrier functions;
- constrained optimization / QP safety filter.

Potential fields alone недостаточны из-за local minima и lack of guarantees.

## 13. Сцена и мир

### 13.1 Геометрия

Сцена должна быть не просто пустым кубом.

Нужны:

- стены;
- пол/потолок;
- колонны/препятствия;
- narrow passages;
- indoor and outdoor presets.

### 13.2 Визуальная текстура

Для VIO/optical flow world должен иметь:

- textured surfaces;
- feature-poor surfaces;
- reflective surfaces;
- repetitive patterns.

Иначе VIO/flow нельзя валидировать.

### 13.3 Динамические объекты

Опционально, но полезно:

- moving obstacles;
- moving targets;
- people/vehicles as occluders.

## 14. Численные методы и расписание

### 14.1 Частоты

Рекомендуемые стартовые частоты:

- physics integrator: 500-1000 Hz;
- motor update: 500-1000 Hz;
- IMU: 200-1000 Hz;
- baro/mag/range/UWB: 5-100 Hz;
- camera/flow: 20-60 Hz;
- estimator: at sensor rates or fused schedule;
- controller: 100-500 Hz;
- swarm planner: 5-50 Hz;
- render: 60 Hz.

### 14.2 Интегратор

Для dynamics:

- RK4 or semi-implicit integrator;
- quaternion normalization after update;
- careful handling of stiff motor dynamics.

Explicit Euler допустим только как временный baseline и только если показано, что шаг достаточно мал и ошибки контролируемы.

### 14.3 Multi-rate execution

Нужен scheduler с накоплением времени, а не "все в каждом кадре".

Каждый subsystem должен иметь:

- own update period;
- own last timestamp;
- deterministic step count.

### 14.4 Rendering separation

Physics должен жить отдельно от rendering.

Рекомендуемая архитектура браузера:

- truth + sensors + estimation in Web Worker;
- heavy numeric kernels в WASM при необходимости;
- UI thread только для render and controls;
- snapshots/interpolated state to renderer.

## 15. Web implementation strategy

### 15.1 Что лучше для браузера

Рекомендуемый стек:

- TypeScript for orchestration and UI;
- Three.js or Babylon.js for rendering;
- Web Workers for simulation loops;
- WASM module for core physics/estimation if CPU cost grows;
- binary logs or structured ring buffers for telemetry.

UI должен быть не просто "панелькой сбоку", а полноценной операторской поверхностью.

Нужны:

- удобная иерархия настроек;
- быстрый доступ к частым параметрам;
- расширенный режим для редких low-level параметров;
- пресеты сценариев и noise profiles;
- возможность сохранять/загружать конфигурации;
- ясная визуальная типографика и читаемая компоновка;
- корректная работа на разных размерах окна.

### 15.2 Что важно для производительности

- struct-of-arrays for many drones;
- preallocated buffers;
- no per-frame object churn;
- deterministic RNG;
- compact telemetry channels;
- spatial partitioning for neighbor search.

Responsive UI не должен означать, что вся логика живет в React render loop или аналогичном UI цикле. Любая интерактивность должна быть отделена от core simulation timing.

### 15.3 Масштабирование на рой

При росте `N` узкие места:

- pairwise collision checks;
- line-of-sight checks;
- UWB/message graph complexity;
- rendering draw calls;
- estimator cost per drone.

Поэтому нужно:

- broad-phase neighbor search;
- bounded communication radius;
- configurable sensor subsets;
- level-of-detail for rendering, but not for active physics agents unless явно оговорено.

## 16. Что считать корректной симуляцией "без GPS"

Ниже список признаков, без которых нельзя утверждать, что реализована GPS-denied swarm simulation:

1. Дрон летает не по true state, а по estimated state.
2. Estimate дрейфует, если сенсоры деградируют.
3. Визуальные сенсоры реально зависят от сцены.
4. UWB реально деградирует в NLOS.
5. Магнитометр реально портится от local EMI и hard/soft iron.
6. Baro реально плавает по drift и airflow.
7. Связь между агентами не идеальна.
8. Formation держится через relative information, а не через hidden global pose.
9. Collision avoidance работает как hard constraint, а не как "надеемся, разойдутся".
10. Есть воспроизводимые сценарии деградации и метрики качества.

## 17. Программа валидации

### 17.1 Single-drone tests

Обязательно реализовать:

- hover trim consistency;
- free-fall sanity check;
- step response in attitude;
- aggressive forward flight with drag;
- wind gust rejection;
- magnetic anomaly response;
- barometer drift response;
- optical flow failure on featureless floor;
- VIO drift in low-feature scene.

### 17.2 Multi-drone tests

Обязательно:

- crossing trajectories;
- corridor passing;
- leader-follower formation;
- formation reconfiguration after one drone dropout;
- packet loss burst;
- UWB NLOS burst;
- collision avoidance with delayed neighbor states.

### 17.3 Monte Carlo

Нужны батч-прогоны по seed-ам с разбросом:

- IMU bias;
- wind intensity;
- sensor dropout rates;
- UWB NLOS probability;
- communication latency;
- payload mass/inertia perturbation.

### 17.4 Acceptance criteria

Для каждой сцены должны логироваться:

- collision count;
- min separation;
- RMS tracking error;
- RMS estimation error;
- drift over horizon;
- innovation consistency;
- saturation time fraction;
- packet loss and delay stats.

## 18. Роадмап реализации

### Phase 0

Скелет проекта, deterministic scheduler, config system, logging.

### Phase 1

Single-drone 6DoF dynamics + motors + basic control.

### Phase 2

IMU + mag + baro + range + optical flow models.

### Phase 3

Error-state EKF and closed-loop GPS-denied hover/trajectory.

### Phase 4

Scene-dependent camera/VIO model and/or UWB relative ranging.

### Phase 5

Multi-drone communication graph + relative localization.

### Phase 6

Formation control + collision avoidance.

### Phase 7

Scenario runner + Monte Carlo + replay + metrics dashboard.

### Phase 8

High-fidelity additions:

- rotor drag;
- turbulence;
- ground/wall effect;
- richer VIO pipeline;
- WMM-driven magnetic field.

## 19. Антипаттерны, которые запрещены

- direct setPosition/setQuaternion from controller;
- one-loop architecture where render drives physics;
- perfect optical flow independent of texture;
- perfect magnetometer independent of EMI;
- ideal UWB with symmetric Gaussian zero-mean errors in NLOS;
- all-to-all instantaneous communication;
- collision avoidance only by visual separation animation;
- magic "stabilize()" functions without modeled states;
- fake SLAM where estimate is just truth plus random noise;
- fake wind as a single random scalar applied to all drones equally;
- UI, в котором нельзя выставить существенные параметры шумов, среды, сенсоров и отказов;
- "современный" интерфейс, который выглядит хорошо, но скрывает реальные состояния системы и ограничения модели.

## 20. Внешние ориентиры и примеры, которые нужно учитывать

Ниже не "копировать как есть", а использовать как инженерные ориентиры.

### Dynamics and control

- Lee, Leok, McClamroch, "Geometric Tracking Control of a Quadrotor UAV on SE(3)".
- Faessler, Franchi, Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories".

### Estimation

- Forster, Carlone, Dellaert, Scaramuzza, "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry".
- Huang, "Visual-Inertial Navigation: A Concise Review".
- ORB-SLAM3 as practical reference for visual/visual-inertial SLAM architecture.

### Simulators

- RotorS: modular Gazebo MAV simulator.
- Flightmare: decoupled rendering + physics, very полезный architectural precedent.
- AirSim: example of combined visual and physical simulation.
- PX4 SITL + Gazebo/jMAVSim: reference for sensor/estimator/controller separation and multi-vehicle flows.

### Swarm and collision avoidance

- Olfati-Saber flocking and consensus-style distributed control.
- ORCA / RVO2-3D as practical reference for decentralized collision avoidance.
- Recent drone swarm relative-positioning papers for GPS-denied pursuit and formation.

### Relative localization and indoor ranging

- UWB survey and NLOS mitigation literature.
- Relative localization via airborne monocular vision for quadrotor swarms.

## 21. Источники

Ниже список источников, использованных для формирования этой спецификации.

1. Visual-Inertial Navigation: A Concise Review
   https://arxiv.org/abs/1906.02650

2. On-Manifold Preintegration for Real-Time Visual-Inertial Odometry
   https://arxiv.org/abs/1512.02363

3. ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM
   https://arxiv.org/abs/2007.11898

4. Geometric Tracking Control of a Quadrotor UAV on SE(3)
   https://doi.org/10.1109/CDC.2010.5717652

5. Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories
   https://arxiv.org/abs/1712.02402

6. Flightmare: A Flexible Quadrotor Simulator
   https://arxiv.org/abs/2009.00563

7. AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles
   https://www.microsoft.com/en-us/research/publication/airsim-high-fidelity-visual-physical-simulation-autonomous-vehicles/

8. PX4 Simulation overview
   https://docs.px4.io/main/en/simulation/

9. PX4 Multi-Vehicle Simulation
   https://docs.px4.io/main/en/simulation/multi-vehicle-simulation

10. PX4 EKF2 tuning and estimator details
    https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf

11. PX4 Optical Flow
    https://docs.px4.io/main/en/sensor/optical_flow.html

12. PX4 Compass Calibration
    https://docs.px4.io/main/en/config/compass.html

13. PX4 Magnetometer hardware and setup
    https://docs.px4.io/main/en/gps_compass/magnetometer

14. Kalibr IMU Noise Model
    https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model

15. NOAA World Magnetic Model, current WMM2025 release
    https://www.ncei.noaa.gov/index.php/products/world-magnetic-model

16. NASA / US Standard Atmosphere references
    https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf

17. NASA atmospheric equations summary
    https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/earth-atmosphere-equation-metric/

18. Dryden turbulence implementation references
    https://ntrs.nasa.gov/citations/19780004071
    https://ntrs.nasa.gov/citations/19980028448

19. A Survey of Robot Swarms' Relative Localization Method
    https://doi.org/10.3390/s22124424

20. Relative Localization within a Quadcopter Unmanned Aerial Vehicle Swarm Based on Airborne Monocular Vision
    https://doi.org/10.3390/drones7100612

21. Drone Swarm Robust Cooperative Formation Pursuit through Relative Positioning in a Location Denial Environment
    https://doi.org/10.3390/drones8090455

22. Survey on NLOS Identification and Error Mitigation for UWB Indoor Positioning
    https://doi.org/10.3390/electronics12071678

23. GNSS-denied UAV indoor navigation with UWB incorporated visual inertial odometry
    https://doi.org/10.1016/j.measurement.2022.112256

24. Optimal Reciprocal Collision Avoidance
    https://gamma-web.iacs.umd.edu/ORCA/

25. RVO2-3D Library
    https://gamma-web.iacs.umd.edu/RVO2/documentation/3d-1.0/

26. Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory
    https://authors.library.caltech.edu/28030/

## 22. Итоговое правило для Claude

Если какой-то subsystem еще не реализован честно, он должен быть помечен как:

- `stub`,
- `simplified`,
- `non-validated`,
- или `not physically faithful yet`.

Никогда не маскируй отсутствие модели красивым UI или общими словами про "swarm intelligence".
