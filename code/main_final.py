import math
import time
import krpc
import numpy as np
import threading
import startLanding
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv

def main():
    with open('../telemetry.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        flag = False
        diff = 0
        turn_start_altitude = 250
        turn_end_altitude = 65000
        target_altitude = 100000
        conn = krpc.connect(name='Launch into orbit')
        vessel = conn.space_center.active_vessel
        srf_frame = vessel.orbit.body.reference_frame
        srf_frame = vessel.orbit.body.reference_frame
        # Set up streams for telemetry
        ut = conn.add_stream(getattr, conn.space_center, 'ut')
        altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
        apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
        stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
        stage_2_fuel = conn.add_stream(stage_2_resources.amount, 'LiquidFuel')
        stage_1_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)
        stage_1_fuel = conn.add_stream(stage_1_resources.amount, 'LiquidFuel')
        speed = list()
        pos = vessel.position(srf_frame)
        canvas = conn.ui.stock_canvas
        screen_size = canvas.rect_transform.size
        panel = canvas.add_panel()
        rect = panel.rect_transform
        rect.size = (300, 200)
        rect.position = (110 - (screen_size[0] / 2), 0)

        text = panel.add_text("Thrust: 0 kN")
        text.rect_transform.position = (0, -20)
        text.color = (1, 1, 1)
        text.size = 18
        text2 = panel.add_text("Speed: 0 m/s")
        text2.rect_transform.position = (0, -40)
        text2.color = (1, 1, 1)
        text2.size = 18
        text3 = panel.add_text("Altitude:")
        text3.rect_transform.position = (0, -60)
        text3.color = (1, 1, 1)
        text3.size = 18
        text4 = panel.add_text("Angle difference:")
        text4.rect_transform.position = (0, -80)
        text4.color = (1, 1, 1)
        text4.size = 18

        # Pre-launch setup
        vessel.control.sas = False
        vessel.control.rcs = False
        vessel.control.throttle = 1.0

        # Countdown...
        print('3...')
        time.sleep(1)
        print('2...')
        time.sleep(1)
        print('1...')
        time.sleep(1)
        print('Launch!')

        # Activate the first stage
        vessel.control.activate_next_stage()
        start_time = time.time()
        vessel.auto_pilot.engage()
        vessel.auto_pilot.target_pitch_and_heading(90, 90)

        # Main ascent loop
        srbs_separated = False
        turn_angle = 0

        while True:
            t = time.time() - start_time
            writer.writerow([f'{vessel.flight(vessel.orbit.body.reference_frame).mean_altitude - 85}',
            f'{t:.3f}',
            f'{vessel.flight(vessel.orbit.body.reference_frame).horizontal_speed}',
            f'{vessel.flight(vessel.orbit.body.reference_frame).vertical_speed}',
            f'{vessel.flight().pitch}'])
            # Gravity turn
            if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
                frac = ((altitude() - turn_start_altitude) /
                        (turn_end_altitude - turn_start_altitude))
                new_turn_angle = frac * 90
                if abs(new_turn_angle - turn_angle) > 0.05:
                    turn_angle = new_turn_angle
                    vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

            # Separate SRBs when finished
            if not srbs_separated:
                print(stage_2_fuel())
                if stage_2_fuel() < 18:
                    vessel.control.throttle = 0.05
                    vessel.control.activate_next_stage()
                    time.sleep(0.125)
                    srbs_separated = True
                    vessel.control.throttle = 0.80
                    vessel.control.activate_next_stage()
                    print('SRBs separated')

            # Decrease throttle when approaching target apoapsis
            if apoapsis() > target_altitude * 0.9:
                print('Approaching target apoapsis')
                break
            time.sleep(0.1)
        # Disable engines when target apoapsis is reached
        vessel.control.throttle = 0.25
        while apoapsis() < target_altitude:
            pass
        print('Target apoapsis reached')
        vessel.control.throttle = 0.0

        # Wait until out of atmosphere
        print('Coasting out of atmosphere')
        while altitude() < 70500:
            pass

        # Plan circularization burn (using vis-viva equation)
        print('Planning circularization burn')
        mu = vessel.orbit.body.gravitational_parameter
        r = vessel.orbit.apoapsis
        a1 = vessel.orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu * ((2. / r) - (1. / a1)))
        v2 = math.sqrt(mu * ((2. / r) - (1. / a2)))
        delta_v = v2 - v1
        node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

        # Calculate burn time (using rocket equation)
        F = vessel.available_thrust
        Isp = vessel.specific_impulse * 9.82
        m0 = vessel.mass
        m1 = m0 / math.exp(delta_v / Isp)
        flow_rate = F / Isp
        burn_time = (m0 - m1) / flow_rate

        # Orientate ship
        print('Orientating ship for circularization burn')
        vessel.auto_pilot.reference_frame = node.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        vessel.control.rcs = True
        vessel.auto_pilot.wait()
        vessel.control.rcs = False

        # Wait until burn
        print('Waiting until circularization burn')
        burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2.)
        lead_time = 5
        conn.space_center.warp_to(burn_ut - lead_time)
        # while ut() < burn_ut - lead_time:
        # if ut() >= burn_ut - lead_time:
        # break
        # time.sleep(0.01)  # Дайте процессору передышку
        # pass  # Keep the script running and monitoring the current time

        # Execute burn
        print('Ready to execute burn')
        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        while time_to_apoapsis() - (burn_time / 2.) > 0:
            pass
        print('Executing burn')
        vessel.control.throttle = 1.0
        time.sleep(burn_time - 0.1)
        print('Fine tuning')
        vessel.control.throttle = 0.05
        remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
        while remaining_burn()[1] > 0.7:
            print(remaining_burn()[1], print(remaining_burn()))
            pass
        vessel.control.throttle = 0
        node.remove()

        mun = conn.space_center.bodies["Mun"]
        vessel.target_body = mun

        # Дождитесь завершения ориентации
        vessel.control.sas = True
        vessel.auto_pilot.disengage()  # Отключаем автопилот
        print('Launch complete')

        conn = krpc.connect(name='Vessel speed')

        # mun = conn.space_center.bodies["Mun"].velocity(obt_frame)
        ap = vessel.auto_pilot
        ap.sas = True
        ap.rcs = True
        ap.sas_mode = ap.sas_mode.prograde
        time.sleep(5)
        angle = 10
        angular_diff = 15

        while abs(angle + angular_diff) >= 0.1:
            obt_frame = vessel.orbit.body.non_rotating_reference_frame
            srf_frame = vessel.orbit.body.reference_frame
            mun_orbit = conn.space_center.bodies["Mun"].position(obt_frame)

            vessel_orbit = vessel.position(obt_frame)
            mun_orb = ((mun_orbit[0] ** 2 + mun_orbit[2] ** 2) ** 0.5)

            # deltaV
            mu = vessel.orbit.body.gravitational_parameter
            r = vessel.orbit.apoapsis
            a1 = vessel.orbit.semi_major_axis
            deltav = math.sqrt(mu / a1) * ((math.sqrt(2 * mun_orb / (mun_orb + a1))) - 1)

            angular_diff = math.pi * ((1 - (1 / (2 * math.sqrt(2))) * math.sqrt((a1 / mun_orb + 1) ** 3)))

            # phase angle
            dot = mun_orbit[0] * vessel_orbit[0] + mun_orbit[2] * vessel_orbit[2]
            det = mun_orbit[0] * vessel_orbit[2] - vessel_orbit[0] * mun_orbit[2]
            angle = math.atan2(det, dot)
            print(deltav)
            # burn time based on engines
            F = vessel.available_thrust
            Isp = vessel.specific_impulse * 9.82
            m0 = vessel.mass
            m1 = m0 / math.exp(deltav / Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate
            diff = angle + angular_diff
            print("Angle Difference:", abs(angle + angular_diff))
            time.sleep(1)
        time.sleep(0.25)
        time.sleep(0.25)
        # Дождитесь завершения ориентации
        vessel.control.sas = False
        vessel.control.rcs = False
        vessel.control.throttle = 1
        time.sleep(burn_time)
        vessel.control.throttle = 0
        time.sleep(2)

        time_to_warp = vessel.orbit.next_orbit.time_to_periapsis + vessel.orbit.time_to_soi_change
        conn.space_center.warp_to(ut() + time_to_warp - 600)
        time2 = ut() + time_to_warp - 600
        # while ut() < (time2):
        # if ut() >= (time2):
        # break
        # time.sleep(0.01)  # Дайте процессору передышку
        # pass  # Keep the script running and monitoring the current time
        conn = krpc.connect(name='Vessel Landing')
        # srf_speed = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')
        # vessel = conn.space_center.active_vessel
        vessel = conn.space_center.active_vessel
        obt_frame = vessel.orbit.body.non_rotating_reference_frame
        orb_speed = conn.add_stream(getattr, vessel.flight(obt_frame), 'speed')
        altitude = conn.add_stream(getattr, vessel.flight(), 'surface_altitude')
        stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
        stage_2_fuel = conn.add_stream(stage_1_resources.amount, 'LiquidFuel')
        print("Deorbiting Vessel...")
        vessel.control.speed_mode = vessel.control.speed_mode.surface
        ap = vessel.auto_pilot
        ap.sas = True
        ap.sas_mode = ap.sas_mode.retrograde
        lunar_frame = vessel.orbit.body.reference_frame
        # ускорение до 50 км

        while altitude() > 40000:
            print(f"Текущая высота: {altitude()} метров")

            # Получаем текущую позицию корабля в системе отсчета Луны
            position = vessel.position(lunar_frame)

            # Нормализуем вектор позиции для вычисления направления вверх относительно поверхности Луны
            norm = np.linalg.norm(position)
            if norm == 0:
                print("Ошибка: длина вектора позиции равна 0")
                break
            up_direction = [coord / norm for coord in position]  # Знак минус для направления "от центра"

            try:
                # Устанавливаем направление автопилота на нормаль к поверхности (вверх относительно Луны)
                vessel.auto_pilot.target_direction = up_direction
                vessel.auto_pilot.reference_frame = lunar_frame
                vessel.auto_pilot.engage()

                # Включаем RCS для коррекции ориентации
                vessel.control.sas = True
                vessel.control.rcs = True

                # Ждем стабилизации (увеличено время ожидания)
                time.sleep(0.5)

                # Проверяем, стабилизировался ли автопилот
                if vessel.auto_pilot.error < 0.1:  # Допустимая ошибка
                    print("Автопилот стабилизировался носом вверх")
                else:
                    print(f"Ошибка автопилота: {vessel.auto_pilot.error}")

            except Exception as e:
                print(f"Ошибка автопилота: {str(e)}")

            print(f"Расстояние от корабля до Луны: {altitude()} метров")

            # Пауза для предотвращения перегрузки цикла
            time.sleep(0.5)
        vessel.auto_pilot.disengage()
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0.0, -1.0, 0.0)  # Point retro-grade surface
        while (orb_speed() > 200) or (int(vessel.orbit.apoapsis_altitude) - int(vessel.flight().surface_altitude)) > 0:
            print(orb_speed())
            print(stage_2_fuel())
            if stage_2_fuel() < 390:
                vessel.control.throttle = 0.0
                time.sleep(0.25)
                vessel.control.activate_next_stage()
                time.sleep(0.25)
                break
            vessel.control.throttle = 0.7
        time.sleep(0.5)
        vessel.control.throttle = 0.0
        time.sleep(0.5)
        vessel.auto_pilot.disengage()
        srbs_separated = True
        vessel.control.activate_next_stage()
        vessel = conn.space_center.active_vessel
        time.sleep(1)
        obt_frame = vessel.orbit.body.non_rotating_reference_frame
        orb_speed = conn.add_stream(getattr, vessel.flight(obt_frame), 'speed')

        space_center = conn.space_center
        vessel = space_center.active_vessel
        # Engage Landing (vertical)
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0.0, -1.0, 0.0)  # Point retro-grade surface
        startLanding.begin_landing(vessel, conn.space_center, conn)

        print("Stabilizing...")
        vessel.auto_pilot.reference_frame = vessel.surface_reference_frame
        vessel.auto_pilot.target_direction = (1.0, 0.0, 0.0)
        time.sleep(10)
        vessel.auto_pilot.disengage()
        vessel.control.sas = True
        ui_thread_stop = True  # Устанавливаем флаг, чтобы остановить поток
        ui_thread2.join()
        print("DONE")

if __name__ == "__main__":
    main()