import math

def get_air_density(alt):
    if alt < 0:
        return 1.225
    if alt > 100000:
        return 0.0
    return 1.225 * math.exp(-alt / 8500.0)

def get_drag_coefficient(mach):
    # Transonic drag divergence spike approximation around Mach 1 
    if mach < 0.8:
        return 0.3
    elif mach < 1.2:
        return 0.65 # Drag spike through the sound barrier(probablly the deal breaker)
    else:
        return 0.25 # Supersonic drop

def run_simulation():
    print("--- ROCKET SIMULATION: MAX Q & ADVANCED DRAG ---")

    try:
        dry_mass = float(input("Enter dry mass (kg) [default 45.0]: ") or 45.0)
        fuel_mass = float(input("Enter fuel mass (kg) [default 250.0]: ") or 250.0)
        payload_mass = float(input("Enter payload mass (kg) [default 10.0]: ") or 10.0)
        thrust = float(input("Enter thrust (N) [default 6000.0]: ") or 6000.0)
        burn_time = float(input("Enter burn time (s) [default 25.0]: ") or 25.0)
        reference_area = float(input("Enter cross-section area (m^2) [default 0.05]: ") or 0.05)
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

    print("\nRunning flight simulation...\n")
    
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

        # i am using a approximation of speed of sound
        sound_speed = 340.0
        mach = abs(vel) / sound_speed
        cd = get_drag_coefficient(mach)

        # dynamicpressure equation according to phy = 0.5 * rho * v^2 (pascals)
        q = 0.5 * rho * vel * abs(vel)
        if q > max_q:
            max_q = q 
            max_q_time = t
            max_q_alt = alt
            max_q_vel = vel

        # aerodynamic drag 
        drag = q * cd * reference_area

        net_force = current_thrust - (current_mass * gravity) - drag
        accel = net_force / current_mass

        vel += accel * dt
        alt += vel * dt
        t += dt

        # stop when rocket reaches apogee
        if vel <= 0 and alt > 10.0:
            break
        if alt < 0:
            alt = 0.0
            break

    print("---------------------------")
    print(f"PEAK APOGEE REACHED: {alt:.1f} m ({alt / 1000.0:.2f} km)")
    print("----------------------------")
    print("MAX Q (Maximum dynamic pressure for your refernce ANALYSIS:")
    print(f" - Peak Q Value  : {max_q:.1f} Pa ({max_q / 1000.0:.2f} kPa)")
    print(f"  - Time         : T+{max_q_time:.2f} s")
    print(f"  - Altitude     : {max_q_alt:.1f} m")
    print(f"  - Velocity     : {max_q_vel:.1f} m/s ")
    print("----------------------------------------")

    if alt >= 100000:
        print("RESULT: SUCCESS! Karman line breached.")
    else:
        print(f"RESULT: FAILED. Short by {(100000 - alt)/1000.0:.2f} km.")

if __name__ == "__main__":
    run_simulation()

        