import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

from gaussian_avoidance import GaussianAvoidanceController


OBSTACLES = [
    (5, 2, 10),
    (-10, 3, 15),
    (20, 3.5, 10),
    (20, 5.5, 7),
    (0, 6, 12),
]

WORLD_OBSTACLES = []

for angle_deg, distance, width_deg in OBSTACLES:
    angle_rad = math.radians(angle_deg)

    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)

    width = distance * math.tan(math.radians(width_deg / 2))

    WORLD_OBSTACLES.append((x, y, width))


def get_visible_obstacles(
    car_x,
    car_y,
    sensor_range=8.0,
    sensor_angle=90,
):
    visible = []

    half_sensor_angle = sensor_angle / 2

    for obs_x, obs_y, width in WORLD_OBSTACLES:
        dx = obs_x - car_x
        dy = obs_y - car_y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        distance_to_edge = distance - width

        if distance_to_edge > sensor_range or dx <= -width:
            continue

        angle_deg = math.degrees(math.atan2(dy, dx))

        obstacle_half_angle = math.degrees(
            math.atan2(width, max(distance, 0.001))
        )

        if abs(angle_deg) <= half_sensor_angle + obstacle_half_angle:
            width_deg = math.degrees(
                2 * math.atan2(width, max(distance, 0.001))
            )

            visible.append((angle_deg, distance, width_deg))

    return visible


CAR_SPEED = 0.08
STEPS = 150

car_x = 0.0
car_y = 0.0

history_x = []
history_y = []

controller = GaussianAvoidanceController(
    safety_distance=1,
    max_steering_deg=40,
    path_length=12,
    points=300,
    lookahead_factor=15,
)

fig, ax = plt.subplots(figsize=(9, 6))


def update(frame):
    global car_x, car_y, history_x, history_y

    # Nach STEPS wieder zurücksetzen
    if frame % STEPS == 0 and frame != 0:
        car_x = 0.0
        car_y = 0.0

        history_x.clear()
        history_y.clear()

    ax.clear()

    visible_obstacles = get_visible_obstacles(
        car_x,
        car_y,
        sensor_range=0.8,
        sensor_angle=180
    )

    old_car_x = car_x

    plan = controller.update(
        current_y=car_y,
        visible_obstacles=visible_obstacles,
        speed=CAR_SPEED,
    )

    car_y = plan.next_y
    car_x += CAR_SPEED

    history_x.append(car_x)
    history_y.append(car_y)

    for obs_x, obs_y, width in WORLD_OBSTACLES:
        circle = plt.Circle((obs_x, obs_y), width, fill=False)
        ax.add_patch(circle)

    ax.scatter(car_x, car_y, s=150, label="Auto")
    ax.plot(history_x, history_y, linewidth=2, label="Gefahrene Route")

    if plan.is_avoiding:
        global_path_x = old_car_x + plan.x_path
        global_path_y = plan.y_path

        ax.plot(
            global_path_x,
            global_path_y,
            linestyle="--",
            label="Berechnete Ausweichroute",
        )

        ax.axhline(
            plan.reference_y,
            linestyle=":",
            linewidth=1,
            label="Ursprüngliche Fahrtrichtung",
        )

    ax.set_title(
        f"Live-Demo | Schritt {frame % STEPS} | "
        f"sichtbare Hindernisse: {len(visible_obstacles)} | "
        f"Lenkwinkel: {plan.steering_angle:.1f}°"
    )

    ax.set_xlim(-1, 12)
    ax.set_ylim(-5, 5)
    ax.set_xlabel("x / Vorwärtsrichtung")
    ax.set_ylabel("y / seitliche Position")
    ax.grid(True)
    ax.legend(loc="upper right")


animation = FuncAnimation(fig, update, frames=None, interval=80)

is_paused = False


def toggle_animation(event):
    global is_paused

    if is_paused:
        animation.event_source.start()
    else:
        animation.event_source.stop()

    is_paused = not is_paused


button_ax = plt.axes([0.8, 0.02, 0.12, 0.05])
button = Button(button_ax, "Stop")
button.on_clicked(toggle_animation)

plt.show()