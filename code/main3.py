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
    with open('telemetryaaaa.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        turn_start_altitude = 250
        turn_end_altitude = 45000
        target_altitude = 400000
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


        # Pre-launch setup
        vessel.control.sas = False
        vessel.control.rcs = False
        vessel.control.throttle = 0.8

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
            writer.writerow([f'{t:.3f}',
            f'{conn.space_center.active_vessel.position(vessel.orbit.body.non_rotating_reference_frame)[0]}',  # X coordinate
            f'{conn.space_center.active_vessel.position(vessel.orbit.body.non_rotating_reference_frame)[1]}',  # Y coordinate
            f'{conn.space_center.active_vessel.position(vessel.orbit.body.non_rotating_reference_frame)[2]}',  # Y coordinate
            f'{vessel.mass}',
            f'{vessel.flight(vessel.orbit.body.non_rotating_reference_frame).mean_altitude}',
            f'{vessel.flight(vessel.orbit.body.non_rotating_reference_frame).horizontal_speed}',
            f'{vessel.flight(vessel.orbit.body.non_rotating_reference_frame).vertical_speed}',
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
                    time.sleep(0.25)
                    srbs_separated = True
                    vessel.control.throttle = 1.00
                    vessel.control.activate_next_stage()
                    print('SRBs separated')

            # Decrease throttle when approaching target apoapsis
            if apoapsis() > target_altitude * 0.95:
                print('Approaching target apoapsis')
                break
            time.sleep(0.01)
            print(conn.space_center.active_vessel.position(vessel.orbit.body.reference_frame))
        # Disable engines when target apoapsis is reached
        vessel.control.throttle = 0.15
        while apoapsis() < target_altitude:
            print(apoapsis(), target_altitude)
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
        burn_time += 2
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
        lead_time = 10
        conn.space_center.warp_to(burn_ut - lead_time)
        # while ut() < burn_ut - lead_time:
        # if ut() >= burn_ut - lead_time:
        # break
        # time.sleep(0.01)  # Дайте процессору передышку
        # pass  # Keep the script running and monitoring the current time

        # Execute burn
        print('Ready to execute burn')
        # Перерасчёт ΔV перед началом манёвра
        r = vessel.orbit.apoapsis
        a1 = vessel.orbit.semi_major_axis
        v1 = math.sqrt(mu * ((2. / r) - (1. / a1)))
        v2 = math.sqrt(mu * ((2. / r) - (1. / a2)))
        delta_v = v2 - v1
        node.prograde = delta_v

        # Лог для проверки перерасчёта
        print(f"Перерасчитанное ΔV: {delta_v}")

        time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
        while time_to_apoapsis() - (burn_time / 2.) > 0:
            pass
        print('Executing burn')

        vessel.control.throttle = 0.5  # Начинаем с 50% тяги
        time.sleep(burn_time / 4)  # Держим 50% четверть времени манёвра
        vessel.control.throttle = 1.0  # Затем переходим на 100% тягу
        time.sleep(burn_time - burn_time / 4 - 0.1)  # Оставшееся время работаем на полной тяге

        print('Fine tuning')
        vessel.control.rcs = True
        vessel.control.throttle = 0.02
        remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
        while remaining_burn()[1] > 2:
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
        vessel.control.throttle = 0
        time.sleep(0.25)
        vessel.control.activate_next_stage()


if __name__ == "__main__":
    main()