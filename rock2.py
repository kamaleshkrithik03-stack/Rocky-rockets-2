import math

def get_air_density(alt):
    if alt < 0:
        return 1.225
    if alt > 100000:
        return 0.0
    return 1.225 * math.exp(-alt / 8500.0)

def get_drag_coefficient(mach):
    # transonic drag spike approximation around mach 1
    if mach < 0.8:
        return 0.3
    elif mach < 1.2:
        return 0.65  # drag spike through the sound barrier (pain to deal with irl)
    else:
        return 0.25  # supersonic drop

def run_simulation():
    print("-------------------------------------------------------------")
    print(" rocket simulation: max q & advanced drag ")
    print("-------------------------------------------------------------")

    try:
        dry_mass = float(input("enter dry mass (kg) [default 45.0]: ") or 45.0)
        fuel_mass = float(input("enter fuel mass (kg) [default 250.0]: ") or 250.0)
        payload_mass = float(input("enter payload mass (kg) [default 10.0]: ") or 10.0)
        thrust = float(input("enter thrust (N) [default 6000.0]: ") or 6000.0)
        burn_time = float(input("enter burn time (s) [default 25.0]: ") or 25.0)
        reference_area = float(input("enter cross-section area (m^2) [default 0.05]: ") or 0.05)
    except ValueError:
        dry_mass, fuel_mass, payload_mass, thrust, burn_time, reference_area = 45.0, 250.0, 10.0, 6000.0, 25.0, 0.05

    mass_flow_rate = fuel_mass / burn_time if burn_time > 0 else 0
    dt = 0.05
    t = 0.0
    alt = 0.0
    vel = 0.0
    current_fuel = fuel_mass

    max_q = 0.0
    max_q_time = 0.0
    max_q_alt = 0.0
    max_q_vel = 0.0

    print("\nrunning flight simulation loop...\n")
    
    while True:
        if current_fuel > 0 and t <= burn_time:
            current_mass = dry_mass + current_fuel + payload_mass
            current_thrust = thrust
            current_fuel -= mass_flow_rate * dt
        else:
            current_mass = dry_mass + payload_mass
            current_thrust = 0.0

        gravity = 9.81
        rho = get_air_density(alt)

        # simple speed of sound approximation
        sound_speed = 340.0
        mach = abs(vel) / sound_speed
        cd = get_drag_coefficient(mach)

        # dynamic pressure equation: q = 0.5 * rho * v^2
        q = 0.5 * rho * vel * abs(vel)
        if q > max_q:
            max_q = q 
            max_q_time = t
            max_q_alt = alt
            max_q_vel = vel

        # aerodynamic drag force
        drag = q * cd * reference_area

        net_force = current_thrust - (current_mass * gravity) - drag
        accel = net_force / current_mass

        vel += accel * dt
        alt += vel * dt
        t += dt

        # stop when rocket peaks out at apogee
        if vel <= 0 and alt > 10.0:
            break
        if alt < 0:
            alt = 0.0
            break

    print("----------------------------------------")
    print(f"peak apogee reached: {alt:.1f} m ({alt / 1000.0:.2f} km)")
    print("----------------------------------------")
    print("max q telemetry analysis:")
    print(f"  - peak q value  : {max_q:.1f} pa ({max_q / 1000.0:.2f} kpa)")
    print(f"  - time          : t+{max_q_time:.2f} s")
    print(f"  - altitude      : {max_q_alt:.1f} m")
    print(f"  - velocity      : {max_q_vel:.1f} m/s")
    print("----------------------------------------")

    if alt >= 100000:
        print("RESULT: success, cleared the karman line.")
    else:
        print(f"RESULT: rip, short by {(100000 - alt)/1000.0:.2f} km.")

if __name__ == "__main__":
    run_simulation()