import math

def get_air_density(alt):
    if alt < 0:
        return 1.225
    if alt > 100000:
        return 0.0
    return 1.225 * math.exp(-alt / 8500.0)

def check_launch_feasibility():
    print("--- karman line basic altitude checker ---")
    
    try:
        dry_mass = float(input("enter rocket dry mass (kg) [default 50.0]: ") or 50.0)
        fuel_mass = float(input("enter propellant fuel mass (kg) [default 200.0]: ") or 200.0)
        payload_mass = float(input("enter payload mass (kg) [default 10.0]: ") or 10.0)
        thrust = float(input("enter engine thrust (N) [default 5000.0]: ") or 5000.0)
        burn_time = float(input("enter burn time (s) [default 20.0]: ") or 20.0)
    except ValueError:
        print("bad input, just using defaults.")
        dry_mass = 50.0
        fuel_mass = 200.0
        payload_mass = 10.0
        thrust = 5000.0
        burn_time = 20.0

    total_initial_mass = dry_mass + fuel_mass + payload_mass
    mass_flow_rate = fuel_mass / burn_time if burn_time > 0 else 0

    dt = 0.05
    t = 0.0
    alt = 0.0
    vel = 0.0
    current_fuel = fuel_mass

    print("\nrunning simple euler integration loop...\n")

    # tracking ascent and coast until it peaks out
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
        drag = 0.5 * rho * vel * abs(vel) * 0.2 * 1.0  

        net_force = current_thrust - (current_mass * gravity) - drag
        accel = net_force / current_mass

        vel += accel * dt
        alt += vel * dt
        t += dt

        # stop when vertical velocity drops to zero 
        if vel <= 0 and alt > 10.0:
            break
        # safety break if it crashes back down instantly
        if alt < 0:
            alt = 0.0
            break

    print("----------------------------------------")
    print(f"initial total mass : {total_initial_mass:.1f} kg")
    print(f"peak apogee reached: {alt:.1f} m ({alt / 1000.0:.2f} km)")
    print("----------------------------------------")

    if alt >= 100000:
        print("RESULT: woo, hit the 100 km karman line.")
    else:
        shortfall = (100000 - alt) / 1000.0
        print(f"RESULT: rip, missed it by {shortfall:.2f} km.")

if __name__ == "__main__":
    check_launch_feasibility()