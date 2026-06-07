-- Автоматическая заготовка сцены CoppeliaSim для ВКР по навигации мобильного робота.
-- Запуск: открыть CoppeliaSim -> Tools/Add-on scripts -> запустить этот файл
-- или выполнить его как Lua script внутри пустой сцены.

local function set_alias(handle, alias)
    if sim.setObjectAlias then
        sim.setObjectAlias(handle, alias, true)
    elseif sim.setObjectName then
        sim.setObjectName(handle, alias)
    end
end

local function set_color(handle, color)
    sim.setShapeColor(handle, nil, sim.colorcomponent_ambient_diffuse, color)
end

local function create_shape(alias, primitive, size, position, color)
    local handle = sim.createPureShape(primitive, 0, size, 0.0)
    set_alias(handle, alias)
    sim.setObjectPosition(handle, -1, position)
    set_color(handle, color)
    return handle
end

local function create_cuboid(alias, size, position, color)
    return create_shape(alias, sim.primitiveshape_cuboid, size, position, color)
end

local function create_cylinder(alias, radius, height, position, color)
    return create_shape(alias, sim.primitiveshape_cylinder, {radius, radius, height}, position, color)
end

local function create_sphere(alias, radius, position, color)
    return create_shape(alias, sim.primitiveshape_spheroid, {radius, radius, radius}, position, color)
end

local function create_path_segment(alias, p1, p2, color)
    local dx = p2[1] - p1[1]
    local dy = p2[2] - p1[2]
    local length = math.sqrt(dx * dx + dy * dy)
    local segment = create_cuboid(alias, {length, 0.035, 0.025}, {(p1[1] + p2[1]) / 2.0, (p1[2] + p2[2]) / 2.0, 0.035}, color)
    sim.setObjectOrientation(segment, -1, {0.0, 0.0, math.atan2(dy, dx)})
    return segment
end

local function add_text_label(alias, text, position)
    local label = sim.addDrawingObject(sim.drawing_text + sim.drawing_overlay, 0.035, 0.0, -1, 1, {0.05, 0.05, 0.05})
    sim.addDrawingObjectItem(label, {position[1], position[2], position[3], text})
    return label
end

-- Очистка предыдущей автоматически созданной сцены: удаляются только объекты с префиксом vkr_;
-- пользовательские модели не затрагиваются.
local all_objects = sim.getObjectsInTree(sim.handle_scene)
for i = 1, #all_objects do
    local ok, alias = pcall(sim.getObjectAlias, all_objects[i], 1)
    if ok and alias and string.sub(alias, 1, 4) == 'vkr_' then
        sim.removeObject(all_objects[i])
    end
end

-- Рабочая плоскость.
local floor = create_cuboid('vkr_floor', {6.0, 6.0, 0.03}, {0.0, 0.0, -0.02}, {0.88, 0.90, 0.92})
add_text_label('vkr_label_floor', 'Плоскость движения', {-2.65, -2.65, 0.08})

-- Стартовая и целевая точки.
local start = create_cylinder('vkr_start_point', 0.18, 0.035, {-2.2, -2.0, 0.035}, {0.0, 0.75, 0.25})
local goal = create_cylinder('vkr_goal_point', 0.18, 0.035, {2.15, 1.8, 0.035}, {0.9, 0.1, 0.1})
add_text_label('vkr_label_start', 'Старт', {-2.45, -2.25, 0.25})
add_text_label('vkr_label_goal', 'Цель', {2.0, 2.05, 0.25})

-- Корпус, колесная база и колеса мобильного робота. Главный объект называется robot,
-- чтобы Python-контроллер coppelia_controller.py мог найти его как /robot.
local robot = create_cuboid('robot', {0.55, 0.38, 0.18}, {-2.2, -2.0, 0.18}, {0.1, 0.35, 0.85})
local wheel_base = create_cuboid('vkr_wheel_base', {0.58, 0.42, 0.05}, {-2.2, -2.0, 0.08}, {0.05, 0.05, 0.05})
local left_wheel = create_cylinder('vkr_left_drive_wheel_encoder', 0.11, 0.06, {-2.2, -2.25, 0.09}, {0.02, 0.02, 0.02})
local right_wheel = create_cylinder('vkr_right_drive_wheel_encoder', 0.11, 0.06, {-2.2, -1.75, 0.09}, {0.02, 0.02, 0.02})
sim.setObjectOrientation(left_wheel, -1, {math.pi / 2.0, 0.0, 0.0})
sim.setObjectOrientation(right_wheel, -1, {math.pi / 2.0, 0.0, 0.0})
sim.setObjectParent(wheel_base, robot, true)
sim.setObjectParent(left_wheel, robot, true)
sim.setObjectParent(right_wheel, robot, true)

-- Сенсорные компоненты локального восприятия.
local lidar = create_cylinder('vkr_lidar_range_sensor', 0.08, 0.12, {-2.2, -2.0, 0.34}, {0.95, 0.75, 0.05})
local front_sensor = create_cuboid('vkr_front_ultrasonic_sensor', {0.08, 0.16, 0.055}, {-1.9, -2.0, 0.20}, {0.0, 0.85, 0.9})
local left_sensor = create_cuboid('vkr_left_infrared_sensor', {0.08, 0.04, 0.055}, {-2.08, -2.22, 0.20}, {0.0, 0.85, 0.9})
local right_sensor = create_cuboid('vkr_right_infrared_sensor', {0.08, 0.04, 0.055}, {-2.08, -1.78, 0.20}, {0.0, 0.85, 0.9})
local imu = create_cuboid('vkr_imu_orientation_sensor', {0.13, 0.10, 0.035}, {-2.2, -2.0, 0.29}, {0.55, 0.0, 0.8})
local contact = create_cuboid('vkr_contact_bumper_sensor', {0.035, 0.34, 0.07}, {-1.90, -2.0, 0.12}, {1.0, 0.45, 0.0})
local left_encoder = create_sphere('vkr_left_wheel_encoder_marker', 0.045, {-2.2, -2.30, 0.17}, {1.0, 1.0, 1.0})
local right_encoder = create_sphere('vkr_right_wheel_encoder_marker', 0.045, {-2.2, -1.70, 0.17}, {1.0, 1.0, 1.0})

local sensors = {lidar, front_sensor, left_sensor, right_sensor, imu, contact, left_encoder, right_encoder}
for i = 1, #sensors do
    sim.setObjectParent(sensors[i], robot, true)
end

-- Локальная карта занятости как условная сетка рядом с роботом.
local grid = create_cuboid('vkr_local_occupancy_grid', {1.25, 1.25, 0.01}, {-1.45, -2.0, 0.012}, {0.72, 0.86, 1.0})
add_text_label('vkr_label_grid', 'Локальная карта / occupancy grid', {-1.85, -1.35, 0.18})
for gx = -3, 3 do
    create_path_segment('vkr_grid_line_x_' .. gx, {-1.45 + gx * 0.18, -2.55}, {-1.45 + gx * 0.18, -1.45}, {0.2, 0.45, 0.9})
    create_path_segment('vkr_grid_line_y_' .. gx, {-2.0, -2.0 + gx * 0.18}, {-0.9, -2.0 + gx * 0.18}, {0.2, 0.45, 0.9})
end

-- Статические препятствия.
create_cuboid('vkr_obstacle_1', {0.55, 0.75, 0.55}, {-0.85, -0.75, 0.275}, {0.55, 0.55, 0.55})
create_cuboid('vkr_obstacle_2', {0.55, 0.55, 0.75}, {0.35, 0.55, 0.375}, {0.45, 0.45, 0.45})
create_cylinder('vkr_obstacle_3', 0.32, 0.65, {1.2, -0.9, 0.325}, {0.60, 0.60, 0.60})
add_text_label('vkr_label_obstacles', 'Статические препятствия', {-0.9, 0.95, 0.65})

-- Маршрут движения A*: точки и сегменты.
local route = {
    {-2.2, -2.0},
    {-1.5, -1.55},
    {-0.65, -1.35},
    {-0.15, -0.35},
    {0.65, 0.15},
    {1.35, 0.95},
    {2.15, 1.8}
}
for i = 1, #route do
    create_sphere('vkr_route_point_' .. i, 0.07, {route[i][1], route[i][2], 0.11}, {1.0, 0.9, 0.0})
    if i > 1 then
        create_path_segment('vkr_route_segment_' .. (i - 1), route[i - 1], route[i], {1.0, 0.65, 0.0})
    end
end
add_text_label('vkr_label_route', 'Маршрут движения A*', {0.55, 1.45, 0.28})

-- Подписи сенсорных компонентов на русском языке.
add_text_label('vkr_label_body', 'Корпус мобильного робота', {-2.9, -2.0, 0.55})
add_text_label('vkr_label_wheels', 'Колеса и энкодеры', {-2.8, -2.55, 0.32})
add_text_label('vkr_label_lidar', 'Лидар / дальномер', {-2.65, -1.55, 0.62})
add_text_label('vkr_label_range', 'УЗ/ИК датчики расстояния', {-1.65, -2.55, 0.46})
add_text_label('vkr_label_imu', 'IMU: положение и ориентация', {-2.95, -1.65, 0.44})
add_text_label('vkr_label_contact', 'Контактный датчик', {-1.75, -1.78, 0.36})

-- Камера для удобного получения скриншота.
local camera = sim.createObject(sim.object_camera_type, 0)
set_alias(camera, 'vkr_screenshot_camera')
sim.setObjectPosition(camera, -1, {0.0, -5.6, 4.2})
sim.setObjectOrientation(camera, -1, {math.rad(58.0), 0.0, 0.0})

print('[VKR CoppeliaSim] Демонстрационная сцена создана: robot, сенсоры, препятствия, старт, цель, маршрут и русскоязычные подписи.')
