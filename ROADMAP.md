# ROADMAP.md

## Назначение

Этот документ превращает [AGENTS.md](./AGENTS.md) и [CLAUDE.md](./CLAUDE.md) в исполнимый план разработки.

Его задача:

- разбить проект на проверяемые фазы;
- задать порядок реализации;
- определить deliverables;
- определить acceptance criteria;
- зафиксировать минимальные тесты и метрики;
- не позволить перескочить через физически важные этапы ради демо.

## Главный принцип

Разработка идет снизу вверх:

1. truth physics;
2. sensors;
3. estimation;
4. control;
5. swarm;
6. safety;
7. scenario runner;
8. polished UI and animation.

Анимация и UI важны, но они надстраиваются над честной симуляцией, а не заменяют ее.

## Целевые рабочие потоки

Работу нужно вести параллельно по 8 потокам:

1. Core simulation
2. Environment and disturbances
3. Sensors
4. Estimation
5. Control and swarm
6. Safety and validation
7. UI/UX and visualization
8. Tooling, reproducibility and performance

Каждая фаза ниже должна явно указывать, какие потоки она затрагивает.

## Предлагаемая структура репозитория

Claude должен стремиться к такой структуре:

```text
src/
  app/
  sim/
    core/
    physics/
    actuators/
    environment/
    sensors/
    estimation/
    control/
    swarm/
    safety/
    scenarios/
    metrics/
    config/
  ui/
    panels/
    overlays/
    charts/
    inspectors/
    controls/
  worker/
  wasm/              # optional later
tests/
  unit/
  integration/
  scenario/
  montecarlo/
docs/
configs/
  drones/
  sensors/
  scenarios/
  noise_profiles/
```

## Артефакты, которые должны появиться рано

До серьезной реализации нужны:

- единая config schema;
- deterministic RNG;
- frame conventions;
- unit system conventions;
- structured logging;
- scenario format;
- result serialization format;
- status labels: `validated`, `simplified`, `stub`, `experimental`.

## Фазы

## Phase 0: Foundation

### Цель

Подготовить каркас проекта, чтобы дальше не ломать архитектуру при добавлении физики и UI.

### Обязательно реализовать

- базовый frontend scaffold;
- simulation worker;
- deterministic scheduler;
- config loader;
- event bus / message channel между worker и UI;
- telemetry pipeline;
- базовую 3D-сцену;
- responsive shell UI;
- пауза, reset, replay seed, step;
- единый модуль систем координат и единиц.

### Deliverables

- приложение запускается локально;
- есть пустой симуляционный цикл с фиксированным timestep;
- UI умеет старт/стоп/reset;
- есть панель конфигурации;
- есть канал логов и метрик;
- есть seedable RNG.

### Acceptance criteria

- одинаковый seed дает одинаковую последовательность внутренних состояний;
- render loop не управляет physics step;
- конфигурация редактируется через UI и/или config files;
- UI остается usable на desktop и узком окне;
- time stepping детерминирован и воспроизводим.

### Минимальные тесты

- deterministic scheduler test;
- config schema validation test;
- frame convention test;
- serialization roundtrip test.

## Phase 1: Single-Drone Truth Physics

### Цель

Построить честную 6DoF-модель одного мультикоптера без estimator-а и без fake position control.

### Обязательно реализовать

- 6DoF rigid-body state;
- translational dynamics;
- rotational dynamics;
- quaternion or SO(3) attitude propagation;
- quadrotor geometry and inertia parameters;
- motor lag;
- thrust and yaw torque model;
- mixer;
- actuator saturation;
- gravity;
- базовая drag model;
- truth-state visualization.

### Deliverables

- один дрон корректно симулируется в 3D;
- есть отображение pose, velocity, attitude, motor speeds;
- можно менять массу, инерцию, arm length, motor constants через UI;
- анимация отражает truth dynamics, а не scripted transforms.

### Acceptance criteria

- free-fall behaves physically plausibly;
- hover trim solution существует при разумных параметрах;
- saturation visibly limits achievable response;
- модель не разваливается численно на реалистичных шагах;
- интеграция устойчива на длинном прогоне.

### Минимальные тесты

- zero-input free-fall test;
- static hover equilibrium test;
- torque sign convention test;
- rotor direction / mixer consistency test;
- long-run numerical stability test.

## Phase 2: Environment and Disturbances

### Цель

Сделать внешнюю среду отдельной подсистемой, а не набором ad hoc поправок.

### Обязательно реализовать

- atmospheric density/pressure baseline;
- mean wind field;
- gust events;
- colored turbulence;
- rotor drag baseline;
- ground effect baseline;
- world geometry with floor/walls/obstacles;
- line-of-sight utilities;
- local magnetic field model baseline.

### Deliverables

- сцены `indoor_basic`, `warehouse`, `open_field`;
- UI-параметры для wind, turbulence, ground effect, magnetic anomaly zones;
- визуальные overlays полей ветра и disturbance zones;
- логирование environmental states.

### Acceptance criteria

- аэродинамика использует air-relative velocity;
- турбулентность не сводится к iid white noise;
- nearby drones see correlated disturbance where expected;
- near-ground behavior отличается от high-altitude hover;
- environmental parameters полностью воспроизводимы по seed и config.

### Минимальные тесты

- wind frame/sign test;
- drag opposes relative air velocity test;
- turbulence reproducibility test;
- ground-effect monotonicity sanity test;
- magnetic-zone perturbation test.

## Phase 3: Sensor Layer Baseline

### Цель

Сделать сенсоры отдельными источниками измерений с честными шумами и временными характеристиками.

### Обязательно реализовать

- IMU;
- magnetometer;
- barometer;
- rangefinder;
- optical flow;
- sensor timestamping;
- latency and jitter;
- bias and random walk;
- dropout and clipping;
- sensor extrinsics.

### Deliverables

- sensor inspector UI;
- live plots raw sensor streams;
- редактор noise profiles;
- включение/выключение отдельных сенсоров;
- визуальная маркировка деградаций и invalid samples.

### Acceptance criteria

- sensor outputs строятся только из truth state и environment;
- разные rates реально поддерживаются;
- sensor noise не внедрен напрямую в controller state;
- optical flow зависит от surface texture и distance;
- magnetometer реагирует на local disturbance;
- UIs позволяют выставлять bias, RW, noise density, latency и dropout.

### Минимальные тесты

- IMU bias random walk test;
- barometer drift trend test;
- range invalid-return test;
- optical flow texture-degradation test;
- magnetometer hard/soft iron transform test.

## Phase 4: Estimation Baseline

### Цель

Перевести single-drone систему на управление по оцененному состоянию в GPS-denied режиме.

### Обязательно реализовать

- error-state EKF or MEKF;
- state propagation from IMU;
- updates from mag/baro/range/flow;
- covariance propagation;
- innovation logging;
- estimator health state;
- estimate vs truth instrumentation.

### Deliverables

- estimator panel;
- covariance/innovation plots;
- truth vs estimate overlays in scene;
- drift metrics dashboard;
- state-source visibility in UI.

### Acceptance criteria

- controller может работать без прямого доступа к truth state;
- при деградации сенсоров estimate реально ухудшается;
- innovations логируются и визуализируются;
- yaw drift possible in expected failure modes;
- bias states converge or drift realistically depending on observability.

### Минимальные тесты

- propagation-only drift test;
- baro + range fusion consistency test;
- mag anomaly innovation rejection test;
- covariance symmetry/PSD sanity test;
- no-truth-leak architecture test.

## Phase 5: Single-Drone Closed-Loop Flight

### Цель

Получить устойчивый GPS-denied полет одного дрона по estimated state.

### Обязательно реализовать

- cascaded controller;
- position/velocity/attitude/rate loops;
- trajectory tracking mode;
- hover mode;
- waypoint mode;
- disturbance rejection hooks;
- failsafe baseline.

### Deliverables

- single-drone mission presets;
- command visualization;
- saturation and error plots;
- trajectory editor baseline;
- manual disturbance injection controls.

### Acceptance criteria

- hover works without hidden perfect state;
- trajectory tracking degrades realistically under estimator drift and wind;
- actuator saturation visible in behavior and logs;
- no controller bypasses motor model;
- system remains numerically stable under scenario resets and replay.

### Минимальные тесты

- hover in calm air;
- hover with IMU bias drift;
- forward trajectory with rotor drag;
- gust rejection test;
- takeoff/landing near ground test.

## Phase 6: Camera/VIO and UWB Aiding

### Цель

Добавить более правдоподобную GPS-denied навигацию для реальных indoor/urban сценариев.

### Обязательно реализовать

- camera model baseline;
- VIO-friendly measurement pipeline or abstracted visual odometry interface;
- UWB range model;
- LOS/NLOS modes;
- inter-sensor timing;
- optional sensor fusion extension points.

### Deliverables

- camera and UWB inspectors;
- NLOS zone visualization;
- scene texture presets;
- feature richness visualization;
- config presets for `imu+baro+flow`, `imu+vio`, `imu+uwb`, `imu+vio+uwb`.

### Acceptance criteria

- VIO/visual mode degrades in feature-poor scenes;
- UWB NLOS introduces asymmetric positive bias or heavy-tail errors;
- relative aiding visibly improves some scenarios and fails in others;
- no visual sensor acts as hidden perfect pose source;
- timing alignment between sensors is explicit and configurable.

### Минимальные тесты

- low-texture VIO degradation test;
- reflective-floor optical flow failure test;
- UWB LOS/NLOS mode-switch test;
- async update timing test;
- fused-estimator fallback test.

## Phase 7: Multi-Drone Core

### Цель

Перейти от single-drone к рою без разрушения архитектурной честности.

### Обязательно реализовать

- multiple drone instances;
- per-drone truth/sensors/estimate/controller;
- neighbor graph;
- communication channels;
- message delays and losses;
- scalable entity management;
- multi-drone scene controls.

### Deliverables

- spawn/remove drones;
- per-drone inspector;
- swarm summary panel;
- communication graph overlay;
- performance panel.

### Acceptance criteria

- each drone keeps isolated truth/estimate state;
- communication is not ideal all-to-all by default;
- neighbor graph updates deterministically;
- performance remains acceptable for target swarm sizes;
- UI stays readable and responsive with multiple drones selected and active.

### Минимальные тесты

- N-drone deterministic spawn test;
- communication delay/loss replay test;
- isolated-state integrity test;
- performance budget smoke test.

## Phase 8: Relative Localization and Formation Control

### Цель

Сделать рой реально GPS-denied, а не просто много независимых дронов.

### Обязательно реализовать

- leader-follower baseline;
- distributed formation mode;
- relative-position-based or range-aided localization;
- formation error metrics;
- reconfiguration after leader failure or link degradation.

### Deliverables

- formation editor or preset selector;
- relative-localization visualization;
- neighbor estimate uncertainty overlays;
- formation quality dashboard.

### Acceptance criteria

- formation control опирается на relative/estimated information;
- degradation in comms/UWB affects formation quality;
- loss of leader or root node is observable and handled;
- formation does not silently use hidden global states;
- users can configure topology and neighbor policies through UI.

### Минимальные тесты

- leader-follower baseline test;
- distributed formation maintenance test;
- leader loss reconfiguration test;
- packet loss burst formation degradation test;
- UWB NLOS formation degradation test.

## Phase 9: Collision Avoidance and Safety

### Цель

Добавить гарантированный safety layer поверх swarm behavior.

### Обязательно реализовать

- separation constraints;
- collision prediction;
- local avoidance algorithm;
- hard safety arbitration;
- no-fly zones;
- emergency stop / loiter / land logic;
- safety event logging.

### Deliverables

- safety overlay;
- predicted collision visualization;
- constraint violation dashboard;
- scenario presets with crossing paths and corridor conflicts.

### Acceptance criteria

- collision avoidance не является просто визуальным repulsion effect;
- safety layer может override swarm commands;
- minimum separation log ведется всегда;
- safety behavior reproduces under same seed;
- near-miss and violation events clearly surfaced in UI and logs.

### Минимальные тесты

- crossing trajectories test;
- narrow corridor test;
- delayed neighbor state avoidance test;
- emergency override arbitration test;
- min-separation invariant smoke test.

## Phase 10: Scenario Runner, Replay and Metrics

### Цель

Превратить симулятор в исследовательский инструмент, а не только interactive demo.

### Обязательно реализовать

- scenario definitions;
- batch runner;
- replay system;
- metrics store;
- export/import configs and results;
- comparison mode across seeds or configs.

### Deliverables

- scenario browser;
- replay timeline;
- side-by-side comparison mode;
- metrics export;
- named experiment presets.

### Acceptance criteria

- любой сценарий можно воспроизвести по seed и config;
- telemetry logs sufficient for audit;
- batch mode и interactive mode используют одну и ту же core simulation;
- metrics comparable across runs;
- UI supports inspecting runs without modifying source code.

### Минимальные тесты

- replay determinism test;
- scenario serialization test;
- metrics export roundtrip test;
- batch-vs-interactive consistency test.

## Phase 11: High-Fidelity Additions

### Цель

Повысить физическую и навигационную правдоподобность после того, как baseline уже честный.

### Приоритетный backlog

- WMM-based magnetic field by location/date;
- improved turbulence model;
- wall/ceiling effect;
- better rotor inflow model;
- battery sag and power limits;
- thermal drift;
- rolling shutter and exposure artifacts;
- dynamic occluders;
- factor-graph or advanced fusion hooks;
- more realistic radio congestion.

### Acceptance criteria

- каждое high-fidelity улучшение имеет отдельный toggle;
- baseline mode не ломается;
- added complexity сопровождается тестами и метриками;
- performance impact измерен и задокументирован.

## UI/UX roadmap

UI должен развиваться не как "последний косметический этап", а параллельно.

### UI Milestone A

- app shell;
- responsive layout;
- simulation controls;
- basic scene;
- config sidebar.

### UI Milestone B

- drone inspector;
- sensor panels;
- charts;
- overlay toggles;
- dark/light or high-contrast readable modes if useful.

### UI Milestone C

- scenario editor;
- swarm topology editor;
- disturbance editor;
- saved presets;
- replay timeline.

### UI Milestone D

- comparison dashboard;
- validation badges;
- expert/advanced parameter mode;
- compact mode for smaller screens.

### UI acceptance rules

- все существенные параметры должны быть discoverable;
- частые controls должны быть доступны быстро;
- редкие low-level параметры допустимо прятать в advanced mode;
- layout должен работать на laptop width и на narrower windows;
- важные графики и статусы не должны перекрываться 3D view;
- современный визуальный стиль не должен мешать читаемости инженерных данных.

## Обязательные метрики по фазам

### Physics

- position, velocity, attitude traces;
- motor commands vs motor speeds;
- saturation ratio;
- energy or power proxy if available.

### Estimation

- RMSE truth vs estimate;
- innovation norms;
- covariance traces;
- bias estimates;
- drift per minute.

### Swarm

- formation error;
- graph connectivity;
- packet delivery ratio;
- delay histogram;
- relative localization error.

### Safety

- minimum separation;
- collision count;
- safety override count;
- time in constraint violation.

### Performance

- physics step time;
- estimator step time;
- render frame time;
- worker/UI utilization proxy;
- real-time factor.

## Definition of Done для каждой фазы

Фаза считается завершенной только если:

- код реализован;
- есть тесты;
- есть сценарий демонстрации;
- есть UI-доступ к relevant parameters;
- есть метрики;
- есть documented limitations;
- нет leakage from truth state into estimator/controller;
- есть replayable seed-based result.

## Что нельзя делать при переходе между фазами

- нельзя вводить временный hidden state shortcut и забывать его убрать;
- нельзя делать новую механику только в UI, без логов и config;
- нельзя пропускать тесты "потом";
- нельзя заменять estimator на truth-plus-noise;
- нельзя выпускать multi-drone phase без communication model;
- нельзя заявлять GPS-denied readiness без long-horizon drift tests.

## Приоритет первой реальной итерации Claude

Если начинать немедленно, первый рабочий цикл должен быть таким:

1. Phase 0 skeleton
2. Phase 1 single-drone truth physics
3. базовый responsive operator UI
4. telemetry + plots
5. первые sanity tests

На этом этапе еще не нужен swarm, но уже нельзя строить архитектуру так, будто swarm потом "как-нибудь добавится".

## Приоритет второй итерации

1. Environment baseline
2. Sensor layer baseline
3. sensor configuration UI
4. disturbance overlays
5. scenario presets for hover and gusts

## Приоритет третьей итерации

1. Estimator baseline
2. closed-loop hover on estimated state
3. truth-vs-estimate visualization
4. drift and innovation dashboard

## Формат задач для Claude

Каждая задача должна включать:

- цель;
- scope;
- out of scope;
- files/modules touched;
- tests to add;
- metrics to expose;
- UI controls to expose;
- expected limitations.

## Формат аудита для Codex

После каждой нетривиальной итерации Codex должен проверять:

1. Есть ли скрытый доступ к truth state.
2. Есть ли физические или временные shortcuts.
3. Выставлены ли важные параметры в UI/config.
4. Пишутся ли метрики и логи.
5. Покрыты ли новые механики тестами.
6. Не сломан ли determinism.
7. Не стала ли анимация красивее за счет деградации валидности.

## Минимальный набор demo-сцен, который должен появиться к baseline

1. Single hover calm air
2. Hover with IMU drift
3. Forward flight with drag and gust
4. Near-ground takeoff and landing
5. Low-texture floor optical flow degradation
6. UWB LOS vs NLOS comparison
7. Two-drone crossing with safety override
8. Four-drone formation with packet loss

## Финальное правило

Если Claude сомневается, что делать дальше, он должен выбирать следующую задачу так:

- сначала закрыть архитектурный долг;
- затем закрыть физическую честность;
- затем закрыть измеримость и метрики;
- затем закрыть UI-доступность параметров;
- и только потом улучшать визуальную полировку.
