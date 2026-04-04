# AGENTS.md

## Назначение

Этот репозиторий должен стать спецификацией и реализацией веб-симулятора полного 3D-полета роя мультикоптеров в GPS-denied режиме. Цель не в "красивой анимации", а в физически, навигационно и алгоритмически правдоподобной модели, пригодной для:

- проверки алгоритмов управления роем;
- проверки алгоритмов навигации без GPS;
- моделирования деградаций сенсоров, среды и связи;
- валидации устойчивости и безопасности;
- последующего переноса логики в реальную робототехническую систему.

Этот файл задает контракт ролей, критерии приемки и режим совместной работы между пользователем, Claude и Codex.

Отдельное требование проекта: несмотря на research-grade характер симулятора, интерфейс и анимация тоже должны быть сильными. UI обязан быть современным, понятным, responsive и достаточно полным, чтобы пользователь мог настраивать все существенные параметры модели, шумы, сенсоры, среду, связь, сценарии и режимы отказов без правки исходников.

## Роли

### Пользователь

Пользователь задает научную и продуктовую цель:

- приоритет реализма против производительности;
- требуемые сценарии;
- допустимые упрощения;
- желаемую архитектуру веб-приложения;
- границы MVP и порядок фаз.

### Claude

Claude отвечает за реализацию.

Claude обязан:

- читать `CLAUDE.md` перед любыми архитектурными решениями;
- не подменять физику кинематической игрушечной моделью;
- не смешивать true state, estimated state и rendered state;
- реализовывать систему по слоям, начиная с single-drone truth simulation;
- закладывать расширяемость для multi-agent, multi-sensor, multi-rate simulation;
- документировать любые допущения явно;
- делать интерфейс не декоративным, а инженерно полезным: понятные панели параметров, диагностические overlays, графики и сценарные controls;
- обеспечивать современный и responsive UI, пригодный для длительной работы с симулятором;
- не скрывать отсутствие реализованной физики за "AI-style" визуализацией.

Claude не должен:

- делать swarm behavior на одной только boids-модели и выдавать это за реалистичный рой БПЛА;
- использовать идеальные сенсоры без bias/drift/latency/dropout;
- давать контроллеру доступ к ground-truth состоянию;
- привязывать архитектуру к GPS как к скрытому костылю;
- прятать важные параметры модели в коде, если они должны быть доступны пользователю как часть симулятора;
- строить все на одном render loop.

### Codex

Codex отвечает за аудит, планирование, валидацию корректности, полноты и научно-инженерной состоятельности.

Codex обязан:

- проверять физическую корректность модели;
- проверять, что estimator наблюдает только то, что реально доступно сенсорам;
- проверять полноту noise/disturbance models;
- проверять согласованность единиц измерения, фреймов, таймстемпов и задержек;
- требовать тесты на идентифицируемость, устойчивость, отказоустойчивость и safety;
- блокировать упрощения, которые ломают валидность симуляции;
- вести чеклист "что уже физически правдоподобно, а что пока stub/mock".

## Необсуждаемые инженерные правила

### 1. Истина, оценка и управление разделяются

В коде и данных обязаны существовать раздельно:

- `truth state`: реальное состояние симулятора;
- `sensor outputs`: noisy, delayed, rate-limited измерения;
- `estimated state`: состояние из фильтра/SLAM/VIO/UWB fusion;
- `controller state`: внутренние состояния регуляторов и планировщиков;
- `render state`: интерполированное состояние для UI.

Ни один контроллер не должен читать `truth state` напрямую.

### 2. GPS по умолчанию отсутствует

Номинальный режим проекта: без GPS.

GNSS допускается только как:

- отдельный ablation mode;
- режим деградации/сравнения;
- инструмент валидации, но не штатный источник состояния.

### 3. Модель должна быть 3D и 6DoF

Для каждого дрона нужна полноценная 6DoF rigid-body dynamics, а не:

- 2D квазикинематика;
- прямое управление позицией;
- "velocity = command";
- телепортация yaw/pitch/roll.

### 4. Симуляция должна быть multi-rate

Минимально требуется разнести по частотам:

- dynamics;
- actuator/motor;
- IMU;
- low-rate sensors;
- estimator;
- guidance/planning;
- render/UI.

### 5. Все важные помехи должны быть явными сущностями модели

Нельзя сваливать все в один Gaussian noise term.

Отдельно моделируются:

- white noise;
- bias;
- bias random walk;
- scale factor error;
- misalignment;
- latency;
- jitter;
- packet loss;
- dropout;
- clipping/saturation;
- environmental disturbances;
- modeling mismatch.

### 6. Determinism обязателен

Любой прогон должен быть воспроизводим при:

- фиксированном seed;
- фиксированной конфигурации сцены;
- фиксированной версии параметров модели;
- фиксированном расписании потоков симуляции.

## Обязательные подсистемы

В проекте должны существовать как самостоятельные модули:

1. `physics`
2. `actuators`
3. `environment`
4. `sensors`
5. `estimation`
6. `communications`
7. `swarm coordination`
8. `guidance and control`
9. `safety and collision avoidance`
10. `scenario runner`
11. `metrics and logging`
12. `rendering and inspection UI`

`rendering and inspection UI` обязан включать:

- современную 3D-визуализацию состояния роя;
- понятные controls запуска, паузы, replay и step-by-step execution;
- полные панели настройки параметров;
- доступ к noise, disturbance, failure и sensor-model параметрам;
- responsive layout для разных размеров экрана;
- визуальную индикацию того, какие подсистемы валидированы, а какие еще `stub` или `simplified`.

## Порядок разработки

Нельзя начинать с красивого multi-drone UI.

Правильный порядок:

1. Single-drone truth dynamics без сенсоров.
2. Motor/propeller/actuator dynamics.
3. Environment and disturbance field.
4. IMU/mag/baro/range/optical-flow/camera/UWB sensor simulation.
5. Error-state estimation stack.
6. Single-drone closed-loop control only from estimated state.
7. Multi-drone communications and relative localization.
8. Formation control and collision avoidance.
9. Scenario matrix and Monte Carlo validation.
10. Browser performance optimization without нарушения физического смысла.

## Уровни реализма

### Level A: Недопустимо как конечная модель

- direct position control;
- boids without flight dynamics;
- ideal sensors;
- no latency;
- no wind;
- no estimator drift;
- no communication effects;
- красивый, но непонятный UI без полного доступа к существенным параметрам симуляции.

### Level B: Допустимо только как временный scaffold

- simple thrust quadratic model;
- linear drag;
- fixed wind field;
- simplified EKF;
- obstacle model without photorealistic rendering.

### Level C: Требуемый baseline

- 6DoF rigid body;
- motor lag and saturation;
- IMU white noise + bias RW;
- barometer drift;
- magnetometer hard/soft iron + local disturbance;
- optical flow with texture/range dependence;
- range sensor with dropout and incidence effects;
- UWB LOS/NLOS modes;
- estimator without ground-truth leakage;
- formation control on estimated/relative state;
- explicit communication delay and packet loss;
- collision avoidance as hard safety layer.

### Level D: High-fidelity target

- rotor drag and aggressive-flight feedforward terms;
- Dryden or von Karman turbulence;
- spatially correlated wind;
- ground/wall/ceiling effect;
- VIO/SLAM with preintegration and loop closure modes;
- magnetic field from WMM plus local anomalies;
- camera artifacts: rolling shutter, blur, exposure changes;
- synchronized scenario playback and Monte Carlo statistics.

## Чеклист приемки для Codex

Codex не должен принимать PR/итерацию, если хотя бы один из пунктов нарушен:

- состояние дрона задается не 6DoF моделью;
- управление использует perfect state;
- sensor model не отделен от truth model;
- estimator не моделирует bias/latency/covariance;
- swarm logic не учитывает связь и относительную локализацию;
- safety сводится только к repulsive potential without guarantees;
- нет сценариев деградации;
- нет unit/system tests на согласованность;
- нет логов innovation/error/constraint violation;
- нет описания ограничений текущей реализации.

## Обязательные метрики

Для каждой важной фазы должны считаться:

- position RMSE truth vs estimate;
- attitude error;
- velocity error;
- bias estimation error;
- control saturation ratio;
- collision count and minimum separation;
- formation error;
- communication packet delivery ratio;
- estimator innovation statistics;
- drift rate over time without loop closure;
- recovery time after disturbance;
- CPU time per subsystem;
- real-time factor.

## Обязательные сценарии валидации

Минимальный набор:

1. Hover in calm air.
2. Hover with IMU bias drift.
3. Hover with magnetic anomaly.
4. Forward flight with rotor drag.
5. Wind gust corridor.
6. Takeoff/landing near ground.
7. Indoor flight with poor visual texture.
8. UWB NLOS degradation.
9. Optical flow failure over reflective/featureless floor.
10. Multi-drone crossing trajectories.
11. Leader loss / packet loss / delay spike.
12. Long-horizon drift in GPS-denied mode.

## Как трактовать "готово"

Фича считается готовой только если:

- реализована;
- покрыта тестом;
- снабжена параметрами;
- пишет метрики;
- имеет документированные ограничения;
- не ломает determinism;
- доступна из UI, если относится к пользовательски настраиваемой симуляционной конфигурации;
- встроена в scenario runner.

## Политика источников

Если реализуется или валидируется:

- динамика;
- оценивание состояния;
- шумовые модели;
- swarm guidance;
- collision avoidance;
- геомагнитная или атмосферная модель;

то решения должны опираться на первичные или официальные источники:

- arXiv/IEEE/RA-L/ICRA/IROS/TRO/CDC papers;
- PX4 official docs;
- NOAA/NASA official technical docs;
- официальные документы библиотек и симуляторов.

## Главный принцип

Если возникает выбор между:

- "быстро показать красивый swarm demo"
- и
- "сохранить корректность модели, наблюдаемость и инженерную правду"

всегда выбирается второе.
