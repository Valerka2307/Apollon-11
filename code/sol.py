import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


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
m_0 = 154360   # Начальная масса ракеты (кг)
m_1 = 85916    # Масса после отделения первой ступени (кг)

fuel_consump_1 = 30.135 + 36.609  # кг/c (ЖТ + Окислитель)
mu_1 = fuel_consump_1 * 4         # Первая ступень
mu_2 = fuel_consump_1             # Вторая ступень

F_thrust_1 = 600 * 4 * 1e3  # Н (Первая ступень)
delta_F_thrust_1 = 38 * 1e3
F_thrust_3 = 4100 * 1e3
delta_F_thrust_3 = - 700 * 1e3
F_thrust_2 = 700 * 1e3     # Н (Вторая ступень)
delta_F_thrust_2 = 8.1 * 1e3

alpha_0 = np.pi / 2  # Начальный угол тангажа (90°)
alpha_1 = np.deg2rad(54.5)  # Угол после первой ступени
delta_alpha = -np.deg2rad(0.4)  # Изменение угла тангажа (первая ступень)
delta_alpha2 = -np.deg2rad(0.55)  # Изменение угла тангажа (вторая ступень)
alpha_min = np.deg2rad(8)        # Мин. угол тангажа
alpha_max = np.deg2rad(350)      # Макс. угол тангажа

first_stage_time = 54.7          # Работа первой ступени (с)
second_stage_time = 192 - 54.7   # Работа второй ступени (с)

d_rocket = 4.9  # Диаметр первой ступени (м)
d_rocket2 = 3.9  # Диаметр второй ступени (м)
S = np.pi * ((d_rocket / 2) ** 2)  # Площадь первой ступени
S2 = np.pi * ((d_rocket2 / 2) ** 2)  # Площадь второй ступени

# Временный спад тяги перед отделением
thrust_drop_time = 3.0  # Время на спад тяги (с)
thrust_drop_factor = 5660  # Насколько снижается тяга перед отделением

def rocket_motion(t, y):
    x, y_pos, vx, vy = y
    h = np.sqrt(x**2 + y_pos**2) - R_earth
    p = p_0 * np.exp(-M_molar_mass * g * h / (R_gas_const * T))
    rho = (p * M_molar_mass) / (R_gas_const * T)
    beta = np.arctan2(x, y_pos)

    # Определяем, на какой ступени ракета
    if t < first_stage_time - thrust_drop_time:  # Первая ступень
        m = m_0 - mu_1 * t
        thrust = F_thrust_1 + delta_F_thrust_1 * t
        S_current = S
        alpha = max(alpha_0 + delta_alpha * t, alpha_min)
    elif first_stage_time - thrust_drop_time <= t <= first_stage_time:  #Спад тяги перед отделением первой ступени
        m = m_0 - mu_1 * t
        thrust = F_thrust_3 + delta_F_thrust_3 * (t - first_stage_time + thrust_drop_time)
        S_current = S
        alpha = max(alpha_0 + delta_alpha * t, alpha_min)
        #print(f't: {t}, first_stage_time: {first_stage_time}, thrust_drop_time: {thrust_drop_time}')
    elif t < (first_stage_time + second_stage_time):  # Вторая ступень
        m = m_1 - mu_2 * (t - first_stage_time)
        thrust = F_thrust_2 + delta_F_thrust_2 * (t - first_stage_time)
        S_current = S2
        alpha = max(alpha_1 + delta_alpha2 * (t - first_stage_time), alpha_min)
    else:  # Двигатели выключены
        m = m_1 - mu_2 * second_stage_time
        thrust = 0
        S_current = S2
        alpha = alpha_min

    # Ограничение угла тангажа
    alpha = np.clip(alpha, alpha_min, alpha_max)

    # Силы
    F_drag = 0.5 * C * rho * np.sqrt(vx**2 + vy**2) * S_current
    F_grav = G * M_earth * m / (h + R_earth)**2

    # Ускорения
    ax = (thrust * np.cos(alpha) - F_drag * np.cos(alpha) - F_grav * np.sin(beta)) / m
    ay = (thrust * np.sin(alpha) - F_drag * np.sin(alpha) - F_grav * np.cos(beta)) / m

    return [vx, vy, ax, ay]

# Начальные условия
x0, y0 = 0, R_earth
vx0, vy0 = 0, 0

t_span = (0, 192)  # Время интегрирования
t_eval = np.linspace(0, 192, 100000)  # Точки, где считаем решение
y0 = [x0, y0, vx0, vy0]

# Решение системы
sol = solve_ivp(rocket_motion, t_span, y0, t_eval=t_eval, method='RK45',
                max_step=0.1,  # максимальный шаг между точками
                atol=1e-8,     # абсолютная погрешность
                rtol=1e-8)     # относительная погрешность

# Читаем телеметрию из KSP
data_altitude = []
dataspeed = []
dataspeed_x = []
dataspeed_y = []
data_t = []
datax = []
datay = []
dataz = []
datamass = []
data_acceleration = []
datapitch = []

with open('C:\\Users\\ValCombucha\\pictures\\telemetryaaaa(отриц).csv') as file:
    for line in file:
        data = list(map(float, line.strip().split(',')))
        altitude = data[0]
        t = data[0]
        speed_x = data[-3]
        speed_y = data[-2]
        dpitch = data[-1]
        # Ограничиваем время 180 секундами

        datax.append(data[1] - 159784.5243943922)
        datay.append(data[2])
        dataz.append(data[3])
        datamass.append(data[4])
        data_altitude.append(data[5])
        dataspeed_x.append(speed_x)
        dataspeed_y.append(speed_y)
        data_t.append(t)
        dataspeed.append(np.sqrt(speed_x**2 + speed_y**2))
        datapitch.append(dpitch)

data_t2 = []
for i in range(10, len(data_t), 10):
    data_acceleration.append((dataspeed[i] - dataspeed[i - 3]) / (data_t[i] - data_t[i - 3]))
    data_t2.append(data_t[i])

#подсчёт угла тангажа
alpha_arr = []
t = 0
t_test = []
alpha0_test = 90
delta_test = -0.35
delta2_test = -0.5

while t < 178:
    if t < first_stage_time:
        alpha_arr.append(np.clip(alpha0_test + t * delta_test, 0, 90))
        t_test.append(t)
    elif t < first_stage_time + second_stage_time:
        alpha_arr.append(np.clip(alpha0_test + delta2_test * t, 0, 90))
        t_test.append(t)
    t += 0.5

# Данные
x = sol.y[0]
y = sol.y[1]
vx = sol.y[2]
vy = sol.y[3]
ax = np.gradient(vx, sol.t)
ay = np.gradient(vy, sol.t)

# Построение графиков
fig, axs = plt.subplots(3, 2, figsize=(12, 12))

# Координата X
axs[0, 0].plot(sol.t, x, label='Координата X (матмодель)', color='r')
axs[0, 0].plot(data_t, -(np.array(datax)), label='Координата X (KSP)', color='b')
axs[0, 0].set_xlabel('Время, с')
axs[0, 0].set_ylabel('X, м')
axs[0, 0].legend()
axs[0, 0].grid()

# Координата Y
axs[0, 1].plot(sol.t, y, label='Координата Y (матмодель)', color='r')
axs[0, 1].plot(data_t, -(np.array(dataz) + 579000) + R_earth, label='Координата Y (KSP)', color='b')
axs[0, 1].set_xlabel('Время, с')
axs[0, 1].set_ylabel('Y, м')
axs[0, 1].legend()
axs[0, 1].grid()

# Скорость по X
axs[1, 0].plot(sol.t, vx, label='Скорость по X (матмодель)', color='r')
axs[1, 0].plot(data_t, np.array(dataspeed_x), label='Скорость по X (KSP)', color='b')
axs[1, 0].set_xlabel('Время, с')
axs[1, 0].set_ylabel('Скорость, м/с')
axs[1, 0].legend()
axs[1, 0].grid()

# Скорость по Y
axs[2, 1].plot(sol.t, vy, label='Скорость по Y(матмодель)', color='r')
axs[2, 1].plot(data_t, dataspeed_y, label='Скорость по Y (KSP)', color='b')
axs[2, 1].set_xlabel('Время, с')
axs[2, 1].set_ylabel('Скорость, м/с')
axs[2, 1].legend()
axs[2, 1].grid()

# Ускорение
axs[2, 0].plot(data_t2, data_acceleration, label='Ускорение (KSP)', color='b')
axs[2, 0].plot(sol.t, np.sqrt(ax**2 + ay**2), label='Ускорение (матмодель)', color='r')
axs[2, 0].set_xlabel('Время, с')
axs[2, 0].set_ylabel('Ускорение, м/с²')
axs[2, 0].legend()
axs[2, 0].grid()

# Координата Y
axs[1, 1].plot(sol.t, np.sqrt(x**2 + y**2) - R_earth, label='Высота Y (матмодель)', color='r')
axs[1, 1].plot(data_t, data_altitude, label='Высота Y (KSP)', color='b')
axs[1, 1].set_xlabel('Время, с')
axs[1, 1].set_ylabel('Y, м')
axs[1, 1].legend()
axs[1, 1].grid()

# Координата Y
#axs[3, 0].plot(t_test, alpha_arr, label='Угол тангажа (матмодель)', color='r')
#axs[3, 0].plot(data_t, datapitch, label='Угол тангажа (KSP)', color='b')
#axs[3, 0].set_xlabel('Время, с')
#axs[3, 0].set_ylabel('Y, м')
#axs[3, 0].legend()
#axs[3, 0].grid()

plt.tight_layout()
plt.show()
