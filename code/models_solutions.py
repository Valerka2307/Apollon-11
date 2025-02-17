from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
import math

# Константы
G = 6.67430e-11
C = 0.5
T = 300
M_earth = 5.9742e24
R_earth = 6470000
M_molar_mass = 0.029
R_gas_const = 8.31
p_0 = 101325
e = np.e
g = 9.81
time_step = 0.001

# Параметры ракеты
m_0 = 154360
m_1 = 93916
fuel_consump_1 = 29.135 + 35.609     # Расход топлива для одного двигателя первой и второй ступени (в кг/c) (ЖТ   + Окислитель)

mu_1 = fuel_consump_1 * 4             # Расход топлива для 4 двигателей первой ступени
mu_2 = fuel_consump_1                 # Расход топлива для 1 двигателя второй ступени

F_thrust_1 = 900 * 4 * 1e3           # Общая тяга первой ступени (в Ньютонах)
F_thrust_2 = 1000 * 1e3               # Общая тяга второй ступени (в Ньютонах)

alpha_0 = np.pi / 2                  # начальный угол тангажа, ракета направлена вертикально вверх (в радианах)
alpha_1 = np.deg2rad(74.5)           # угол тангажа после отделения первой ступени, взят из симуляции KSP (в радианах)
delta_alpha = -np.deg2rad(0.45)       # скорость изменения угла тангада (в рад/с)
alpha_min = np.deg2rad(0)             # минимальный угол тангажа по KSP, равен 10° (в радианах)
alpha_max = np.deg2rad(350)           # минимальный угол тангажа по KSP, равен 350° = -10° (в радианах)

first_stage_time = 54.7             # Время работы первой ступени (в секундах)
second_stage_time = 207 - 54.7   # Время работы второй ступени (в секундах)

d_rocket = 4.9 # Диаметр ракеты (в метрах)
d_rocket2 = 3.9 # Диаметр ракеты (2 ступени) (в метрах)
S = np.pi * ((d_rocket / 2) ** 2)    # Площадь поперечного сечения ракеты (в м^2)
S2 = np.pi * ((d_rocket2 / 2) ** 2)  # Площадь поперечного сечения ракеты (2 ступени) (в м^2)


def calc_current_mass(default_mass, fuel_consumption, time_passed_from_burn_start):
    return default_mass - fuel_consumption * time_passed_from_burn_start


def calc_gravitation_force(rocket_mass, altitude):
    return (G * M_earth * rocket_mass) / ((R_earth + altitude) ** 2)


def calc_air_resistance_force1(altitude, speed):
    def calc_atmospheric_pressure():
        exponent = (-M_molar_mass * g * altitude) / (R_gas_const * T)
        if exponent < -700:
            return 0
        return p_0 * np.exp(exponent)

    def calc_air_density():
        return (calc_atmospheric_pressure() * M_molar_mass) / (R_gas_const * T)

    return (C * S * (speed ** 2) * calc_air_density()) / 2

def calc_air_resistance_force2(altitude, speed):
    def calc_atmospheric_pressure():
        exponent = (-M_molar_mass * g * altitude) / (R_gas_const * T)
        if exponent < -700:
            return 0
        return p_0 * np.exp(exponent)

    def calc_air_density():
        return (calc_atmospheric_pressure() * M_molar_mass) / (R_gas_const * T)

    return (C * S2 * (speed ** 2) * calc_air_density()) / 2

def derivatives_stage_1(time, altitude, speed_Ox, speed_Oy):
    # Масса и "сырой" угол
    mass = calc_current_mass(m_0, mu_1, time)
    alpha = np.clip(alpha_0 + delta_alpha * time, alpha_min, alpha_max)

    # Скорость и высота
    speed = np.sqrt(speed_Ox**2 + speed_Oy**2)

    air_resistance = calc_air_resistance_force1(altitude, speed)

    # Силы сопротивления воздуха (по Ox и Oy)
    F_air_res_x = air_resistance * np.cos(alpha)
    F_air_res_y = air_resistance * np.sin(alpha)

    # Сила гравитации
    F_grav = calc_gravitation_force(mass, altitude)

    # Производные скоростей (скорости по осям Ox и Oy)
    dvxdt = (F_thrust_1 * np.cos(alpha) - F_air_res_x) / mass
    dvydt = (F_thrust_1 * np.sin(alpha) - F_air_res_y - F_grav) / mass

    return dvxdt, dvydt


def derivatives_stage_2(time, altitude, speed_Ox, speed_Oy):
    # Рассчитываем массу ракеты и угол тангажа на текущее время
    mass = calc_current_mass(m_1, mu_2, time)
    alpha = np.clip(alpha_1 + delta_alpha * time, alpha_min, alpha_max)

    speed = np.sqrt(speed_Ox ** 2 + speed_Oy ** 2)

    # Сила сопротивления воздуха
    air_resistance = calc_air_resistance_force2(altitude, speed)

    # Силы сопротивления воздуха (по Ox и Oy)
    F_air_res_x = air_resistance * np.cos(alpha)
    F_air_res_y = air_resistance * np.sin(alpha)

    # Сила гравитации
    F_grav = calc_gravitation_force(mass, altitude)

    # Производные скоростей (скорости по осям Ox и Oy)
    dvxdt = (F_thrust_2 * np.cos(alpha) - F_air_res_x) / mass
    dvydt = (F_thrust_2 * np.sin(alpha) - F_air_res_y - F_grav) / mass

    return dvxdt, dvydt


# Читаем телеметрию из KSP
dataaltitude = []
dataspeed = []
dataspeed_x = []
dataspeed_y = []
data_t = []

with open('telemetry2.csv') as file:
    for line in file:
        data = list(map(float, line.strip().split(',')))
        altitude = data[0]
        t = data[1]
        speed_x = data[-3]
        speed_y = data[-2]

        # Ограничиваем время 180 секундами

        dataaltitude.append(altitude)
        dataspeed_x.append(speed_x)
        dataspeed_y.append(speed_y)
        data_t.append(t)
        dataspeed.append(math.sqrt(speed_x ** 2 + speed_y ** 2))


# Первая ступень
def stage_1(t, y):
    altitude, vx, vy = y
    dvxdt, dvydt = derivatives_stage_1(t, altitude, vx, vy)
    return [vy, dvxdt, dvydt]


# Вторая ступень
def stage_2(t, y):
    altitude, vx, vy = y
    dvxdt, dvydt = derivatives_stage_2(t, altitude, vx, vy)
    return [vy, dvxdt, dvydt]


# Решаем для первой ступени с повышенной точностью
sol1 = solve_ivp(
    stage_1,
    [0, first_stage_time],
    [0, 0, 0],  # Начальная высота, Vx, Vy
    t_eval=np.arange(0, first_stage_time, time_step),
    method='DOP853',  # Более точный метод интегрирования
    rtol=1e-8,        # Относительная погрешность
    atol=1e-10        # Абсолютная погрешность
)

# Начальные условия для второй ступени
y0_stage2 = [sol1.y[0, -1], sol1.y[1, -1], sol1.y[2, -1]]

# Решаем для второй ступени с теми же параметрами точности, ограничивая время до 181 секунды
time_limit = 181 - first_stage_time
sol2 = solve_ivp(
    stage_2,
    [0, min(second_stage_time, time_limit)],
    [sol1.y[0, -1], sol1.y[1, -1], sol1.y[2, -1]],
    t_eval=np.arange(0, min(second_stage_time, time_limit), time_step),
    method='DOP853',
    rtol=1e-8,
    atol=1e-10
)

# Объединяем результаты
t_vals = np.concatenate((sol1.t, sol2.t + first_stage_time))
y_vals = np.concatenate((sol1.y[0], sol2.y[0]))
vx_vals = np.concatenate((sol1.y[1], sol2.y[1]))
vy_vals = np.concatenate((sol1.y[2], sol2.y[2]))
v_vals = np.sqrt(vx_vals ** 2 + vy_vals ** 2)

# Интерполяция расчетных данных под телеметрию
y_vals_intp = np.interp(data_t, t_vals, y_vals)
v_vals_intp = np.interp(data_t, t_vals, v_vals)
vx_vals_intp = np.interp(data_t, t_vals, vx_vals)
vy_vals_intp = np.interp(data_t, t_vals, vy_vals)

# Разница между телеметрией и расчетами
dataaltitude_diff = np.subtract(dataaltitude, y_vals_intp)
dataspeed_diff = np.subtract(dataspeed, v_vals_intp)
dataspeed_x_diff = np.subtract(dataspeed_x, vx_vals_intp)
dataspeed_y_diff = np.subtract(dataspeed_y, vy_vals_intp)

# Построение графиков
plt.figure(figsize=(14, 20))

# График высоты
plt.subplot(5, 1, 1)
plt.plot(data_t, np.array(dataaltitude) / 1000, label='Телеметрия KSP', color='green')
plt.plot(t_vals, y_vals / 1000, label='Модельная высота', color='blue', linestyle='--')
plt.xlabel('Время (с)')
plt.ylabel('Высота (км)')
plt.title('Изменение высоты ракеты')
plt.grid(True)
plt.legend()

# График скорости
plt.subplot(5, 1, 2)
plt.plot(data_t, dataspeed, label='Телеметрия KSP', color='purple')
plt.plot(t_vals, v_vals, label='Модельная скорость', color='red', linestyle='--')
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/с)')
plt.title('Изменение скорости ракеты')
plt.grid(True)
plt.legend()

# График скорости по X
plt.subplot(5, 1, 3)
plt.plot(data_t, dataspeed_x, label='Телеметрия KSP по X', color='brown')
plt.plot(t_vals, vx_vals, label='Модельная скорость по X', color='orange', linestyle='--')
plt.xlabel('Время (с)')
plt.ylabel('Скорость по X (м/с)')
plt.title('Изменение скорости по X')
plt.grid(True)
plt.legend()

# График скорости по Y
plt.subplot(5, 1, 4)
plt.plot(data_t, dataspeed_y, label='Телеметрия KSP по Y', color='magenta')
plt.plot(t_vals, vy_vals, label='Модельная скорость по Y', color='g', linestyle='--')
plt.xlabel('Время (с)')
plt.ylabel('Скорость по Y (м/с)')
plt.title('Изменение скорости по Y')
plt.grid(True)
plt.legend()

# Разница между моделью и телеметрией
plt.subplot(5, 1, 5)
plt.plot(data_t, dataaltitude_diff / 1000, label='Разница в высоте', color='blue')
plt.plot(data_t, dataspeed_diff, label='Разница в скорости', color='red')
plt.plot(data_t, dataspeed_x_diff, label='Разница в скорости по X', color='orange')
plt.plot(data_t, dataspeed_y_diff, label='Разница в скорости по Y', color='g')
plt.xlabel('Время (с)')
plt.ylabel('Разница (км, м/с)')
plt.title('Разница между расчетами и телеметрией')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
