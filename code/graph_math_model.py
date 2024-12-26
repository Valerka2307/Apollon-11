import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Константы
G = 6.674e-11  # гравитационная постоянная, м^3/(кг*с^2)
M = 5.972e24   # масса Земли, кг
R_earth = 6371e3  # радиус Земли, м
rho0 = 1.225   # плотность воздуха на уровне моря, кг/м^3
H = 8500       # масштаб высоты, м
Cd = 0.5       # коэффициент сопротивления
A = 10         # площадь сечения ракеты, м^2
T = 35e6       # сила тяги, Н
m0 = 2.8e6     # начальная масса ракеты, кг
dm_dt = -2500  # скорость сгорания топлива, кг/с

# Функция плотности воздуха
def rho(r):
    h = r - R_earth  # высота над поверхностью Земли
    return rho0 * np.exp(-h / H) if h >= 0 else 0

# Сила сопротивления
def drag(v, r):
    return 0.5 * rho(r) * Cd * A * v**2

# Ускорение свободного падения
def g(r):
    return G * M / r**2

# Система дифференциальных уравнений
def rocket_ode(t, y):
    r, v, m = y  # распаковка переменных: r - радиус, v - скорость, m - масса
    if m > 1e5:  # двигатели работают только до критической массы
        thrust = T
        dm = dm_dt
    else:
        thrust = 0
        dm = 0
    dv_dt = thrust / m - g(r) - drag(v, r) / m  # изменение скорости
    dr_dt = v                                   # изменение высоты
    return [dr_dt, dv_dt, dm]

# Начальные условия
r0 = R_earth  # старт с поверхности Земли
v0 = 0        # начальная скорость
y0 = [r0, v0, m0]

# Решение системы ОДУ
t_span = (0, 500)  # временной интервал в секундах
t_eval = np.linspace(t_span[0], t_span[1], 1000)  # точки для расчета
solution = solve_ivp(rocket_ode, t_span, y0, t_eval=t_eval, method='RK45')

# Распаковка решений
r_sol = solution.y[0]  # радиус
v_sol = solution.y[1]  # скорость
m_sol = solution.y[2]  # масса
t_sol = solution.t     # время

# Перевод радиуса в высоту над поверхностью Земли
h_sol = r_sol - R_earth

def rocket_ode_with_orbit(t, y):
    r, v, m = y  # распаковка переменных: r - радиус, v - скорость, m - масса
    if m > 1e5:  # двигатели работают только до критической массы
        thrust = T
        dm = dm_dt
    else:
        thrust = 0
        dm = 0

    # Условие достижения орбиты: высота более 200 км и скорость близка к орбитальной
    h = r - R_earth
    v_orbit = np.sqrt(G * M / r)
    if h > 200e3 and abs(v - v_orbit) < 50:
        thrust = 0  # двигатели выключаются
        dm = 0      # масса перестает изменяться
        v = v_orbit  # стабилизация скорости

    dv_dt = thrust / m - g(r) - drag(v, r) / m  # изменение скорости
    dr_dt = v                                   # изменение высоты
    return [dr_dt, dv_dt, dm]

# Решение скорректированной системы ОДУ
solution_orbit = solve_ivp(rocket_ode_with_orbit, t_span, y0, t_eval=t_eval, method='RK45')

# Распаковка решений
r_sol_orbit = solution_orbit.y[0]  # радиус
v_sol_orbit = solution_orbit.y[1]  # скорость
m_sol_orbit = solution_orbit.y[2]  # масса
t_sol_orbit = solution_orbit.t     # время

# Перевод радиуса в высоту над поверхностью Земли
h_sol_orbit = r_sol_orbit - R_earth

# Пересчет ускорения
a_sol_orbit = np.gradient(v_sol_orbit, t_sol_orbit)

# Построение обновленных графиков
plt.figure(figsize=(16, 12))

# График высоты
plt.subplot(4, 1, 1)
plt.plot(t_sol_orbit, h_sol_orbit / 1e3, label="Высота (км)")
plt.xlabel("Время (с)")
plt.ylabel("Высота (км)")
plt.title("Изменение высоты ракеты с течением времени (с учетом орбиты)")
plt.grid()
plt.legend()

# График скорости
plt.subplot(4, 1, 2)
plt.plot(t_sol_orbit, v_sol_orbit, label="Скорость (м/с)", color="orange")
plt.axhline(y=np.sqrt(G * M / (R_earth + 200e3)), color='r', linestyle='--', label="Орбитальная скорость")
plt.xlabel("Время (с)")
plt.ylabel("Скорость (м/с)")
plt.title("Изменение скорости ракеты с течением времени (с учетом орбиты)")
plt.grid()
plt.legend()

# График массы
plt.subplot(4, 1, 3)
plt.plot(t_sol_orbit, m_sol_orbit / 1e3, label="Масса (тонн)", color="green")
plt.xlabel("Время (с)")
plt.ylabel("Масса (тонн)")
plt.title("Изменение массы ракеты с течением времени (с учетом орбиты)")
plt.grid()
plt.legend()

# График ускорения
plt.subplot(4, 1, 4)
plt.plot(t_sol_orbit, a_sol_orbit, label="Ускорение (м/с²)", color="red")
plt.xlabel("Время (с)")
plt.ylabel("Ускорение (м/с²)")
plt.title("Изменение ускорения ракеты с течением времени (с учетом орбиты)")
plt.grid()
plt.legend()

plt.tight_layout()
plt.show()
