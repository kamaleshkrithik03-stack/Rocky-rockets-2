import math

def calculate_air_density(alt, temp_c, humidity_pct):
    if alt < 0:
        alt = 0.0
    if alt > 100000:
        return 0.0

    # Convert Celsius to Kelvin
    T_k = temp_c + 273.15

    # Standard sea level pressure (Pa) and gas constant for dry air
    P_0 = 101325.0
    R_d = 287.05

    # Remember pressure drops exponentially with altitude (scale height is 8500m)
    pressure = P_0 * math.exp(-alt / 8500.0)

    # Saturation vapor pressure
    es = 610.78 * math.exp((17.27 * temp_c) / (temp_c + 237.3))
    # Actual vapor pressure
    e = es * (humidity_pct / 100.0)

    # Virtual temperature accounting for humidity (moist air is less dense)
    # Tv = T / (1- (e / P) * (1- Mw/Md)) where Mw/Md ~ 0.622
    p_total = pressure
    if p_total > e:
        tv = T_k / (1.0 - (e / p_total) * (1.0 - 0.622))
    else:
        tv = T_k

    # ideal gas law for moist air density
    density = (pressure - (0.378 * e)) / (R_d * tv)
    return max(density, 0.0)

def get_wind_velocity(alt, base_wind_speed, wind_direction_mode):
    # Wind shear profile: wind increases with altitude (jet stream effect)
    altitude_multiplier = 1.0 + (alt / 10000.0) if alt < 15000 else 2.5
    current_wind = base_wind_speed * altitude_multiplier

    # Direction handling: 1 for Tailwind (pushes down), -1 for headwind
    direction_sign = 1.0 if wind_direction_mode.lower() in ["tailwind", "east", "following"] else -1.0

    return current_wind * direction_sign

def get_drag_coefficient(mach):
    if mach < 0.8:
        return 0.3
    elif mach < 1.2:
        return 0.65 # Transonic drag spike
    else:
        return 0.25

def run_environmental_simulation():
    print("--- ROCKET SIMULATION: 2D TRAJECTORY, ATMOSPHERE & DYNAMIC WIND ---")

    try:
        # Rocket parameters
        dry_mass = float(input("Enter dry mass (kg) [default 45.0]: ") or 45.0)
        fuel_mass = float(input("Enter ascent fuel mass (kg) [default 250.0]: ") or 250.0)
        landing_fuel = float(input("Enter landing reserves (kg) [default 30.0]: ") or 30.0)
        payload_mass = float(input("Enter payload mass (kg) [default 10.0]: ") or 10.0)
        thrust = float(input("Enter ascent thrust (N) [default 6000.0]: ") or 6000.0)
        landing_thrust = float(input("Enter landing thrust (N) [default 4500.0]: ") or 4500.0)
        burn_time = float(input("Enter ascent burn time (s) [default 25.0]: ") or 25.0)
        reference_area = float(input("Enter cross-section area (m^2) [default 0.05]: ") or 0.05)

        # Environmental inputs
        print("\n--- Environmental & Weather Parameters (As close as a real rocket launch sim!!) ---")
        temp_c = float(input("Enter ground ambient temperature (°C) [default 25.0]: ") or 25.0)
        humidity_pct = float(input("Enter relative humidity (%) [default 60.0]: ") or 60.0)
        base_wind_speed = float(input("Enter base wind speed (m/s) [default 10.0]: ") or 10.0)
        wind_dir = input("Enter wind direction ('Tailwind' or 'Headwind') [default Tailwind]: ") or "Tailwind"

    except ValueError:
        dry_mass, fuel_mass, landing_fuel, payload_mass = 45.0, 250.0, 30.0, 10.0
        thrust, landing_thrust, burn_time, reference_area = 6000.0, 4500.0, 25.0, 0.05
        temp_c, humidity_pct, base_wind_speed, wind_dir = 25.0, 60.0, 10.0, "Tailwind"

    mass_flow_rate = fuel_mass / burn_time if burn_time > 0 else 0
    landing_burn_rate = 5.0

    dt = 0.05
    t = 0.0
    x = 0.0
    y = 0.0
    vx = 0.0
    vy = 0.0

    current_fuel = fuel_mass
    current_landing_fuel = landing_fuel

    max_q = 0.0
    max_drift = 0.0
    apogee = 0.0
    phase = "ASCENT"

    print("\nRunning flight simulation with the dynamic atmospheric conditions...\n")

    while y >= 0.0 or phase in ["ASCENT", "COAST", "DESCENT", "LANDING_BURN"]:
        gravity = 9.81

        # Dynamic atmosphere calculations
        rho = calculate_air_density(y, temp_c, humidity_pct)
        wind_vx = get_wind_velocity(y, base_wind_speed, wind_dir)

        # Phase management & mass/thrust logic
        if phase == "ASCENT":
            if current_fuel > 0 and t <= burn_time:
                current_mass = dry_mass + current_fuel + current_landing_fuel + payload_mass
                current_thrust = thrust
                current_fuel -= mass_flow_rate * dt
            else:
                current_mass = dry_mass + current_landing_fuel + payload_mass
                current_thrust = 0.0
                phase = "COAST"
        elif phase == "COAST":
            current_mass = dry_mass + current_landing_fuel + payload_mass
            current_thrust = 0.0
            if vy <= 0:
                apogee = y
                phase = "DESCENT"
        elif phase == "DESCENT":
            current_mass = dry_mass + current_landing_fuel + payload_mass
            current_thrust = 0.0
            if y < 1500.0 and vy < -15.0 and current_landing_fuel > 0:
                phase = "LANDING_BURN"
        elif phase == "LANDING_BURN":
            if current_landing_fuel > 0 and y > 0:
                current_mass = dry_mass + current_landing_fuel + payload_mass
                current_thrust = landing_thrust
                current_landing_fuel -= landing_burn_rate * dt
            else:
                current_thrust = 0.0
                phase = "FREE FALL"
        else:
            current_mass = dry_mass + payload_mass
            current_thrust = 0.0

        # Gravity turn pitch program
        if phase == "ASCENT":
            pitch_angle = min(math.radians(30.0), t * 0.015)
        else:
            pitch_angle = 0.0
        thrust_x = current_thrust * math.sin(pitch_angle)
        thrust_y = current_thrust * math.cos(pitch_angle)

        # Relative airspeed vector against dynamic wind
        v_rel_x = vx - wind_vx
        v_rel_y = vy
        v_rel = math.sqrt(v_rel_x**2 + v_rel_y**2)

        sound_speed = 340.0
        mach = v_rel / sound_speed
        cd = get_drag_coefficient(mach)

        q = 0.5 * rho * v_rel**2
        if q > max_q:
            max_q = q
        
        # Drag forces opposing relative motion
        drag_force = q * cd * reference_area
        drag_x = 0.0
        drag_y = 0.0
        if v_rel > 0.001:
            drag_x = drag_force * (v_rel_x / v_rel)
            drag_y = drag_force * (v_rel_y / v_rel)

        # Equation of motion (finally can apply what I learnt in school fr)
        accel_x = (thrust_x - drag_x) / current_mass
        accel_y = (thrust_y - (current_mass * gravity) - drag_y) / current_mass

        vx += accel_x * dt
        vy += accel_y * dt
        x += vx * dt
        y += vy * dt
        t += dt

        if y > apogee and phase in ["ASCENT", "COAST"]:
            apogee = y
        if abs(x) > max_drift:
            max_drift = abs(x)

        if y <= 0.0:
            y = 0.0
            break

    print("------------------------------------------")
    print(f"ENVIRONMENT SUMMARY:")
    print(f"  - Ground Temp      : {temp_c}°C | Humidity: {humidity_pct}%")
    print(f"  - Wind Profile     : {base_wind_speed} m/s ({wind_dir})")
    print("------------------------------------------")
    print(f"PEAK APOGEE REACHED : {apogee:.1f} m ({apogee / 1000.0:.2f} km)")
    print(f"MAX DYNAMIC PRESSURE: {max_q:.1f} Pa ({max_q / 1000.0:.2f} kPa)")
    print(f"MAX HORIZONTAL DRIFT: {max_drift:.1f} m")
    print(f"TOUCHDOWN VELOCITY  : {abs(vy):.2f} m/s")
    print("------------------------------------------")

    if abs(vy) < 5.0:
        print("RESULT: SUCCESS! Soft landing achieved under custom atmospheric settings as well. You are now ready to fly irl!")
    else:
        print("RESULT: HARD LANDING / CRASH.")

if __name__ == "__main__":
    run_environmental_simulation()