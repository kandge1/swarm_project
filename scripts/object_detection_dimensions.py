import cv2
import numpy as np
import time
import math


# =========================================================
# SETTINGS
# =========================================================

CAMERA_DEVICE = "/dev/video0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Ignore small foreground changes and small coloured spots.
MIN_OBJECT_AREA = 1500
MIN_OBJECT_WIDTH = 25
MIN_OBJECT_HEIGHT = 25

# Increase this to ignore shadows and small lighting changes.
# Reduce it if real objects appear incomplete in the foreground mask.
BACKGROUND_THRESHOLD = 30

# Prevent almost the whole frame from being treated as one object.
MAX_OBJECT_AREA_RATIO = 0.70

# Enter the true visible width of your reference object.
# Example: a card or block that is exactly 2 inches wide.
KNOWN_REFERENCE_WIDTH_INCHES = 2.0

# Number of readings used for the final average.
MEASUREMENTS_TO_AVERAGE = 10

# Calculated when C is pressed.
pixels_per_inch = None

# Measurement state.
measurement_samples = []
final_measurement = None
last_measurement_type = None


# =========================================================
# COLOUR RANGES IN HSV
# =========================================================

COLOR_RANGES = {
    "Red": [
        (
            np.array([0, 90, 55]),
            np.array([10, 255, 255])
        ),
        (
            np.array([170, 90, 55]),
            np.array([179, 255, 255])
        )
    ],

    "Orange": [
        (
            np.array([8, 90, 60]),
            np.array([20, 255, 255])
        )
    ],

    "Yellow": [
        (
            np.array([20, 75, 75]),
            np.array([36, 255, 255])
        )
    ],

    "Green": [
        (
            np.array([36, 45, 35]),
            np.array([90, 255, 255])
        )
    ],

    "Cyan": [
        (
            np.array([80, 40, 40]),
            np.array([100, 255, 255])
        )
    ],

    "Blue": [
        (
            np.array([100, 55, 35]),
            np.array([130, 255, 255])
        )
    ],

    "Purple": [
        (
            np.array([130, 40, 35]),
            np.array([165, 255, 255])
        )
    ],

    "Pink": [
        (
            np.array([160, 30, 70]),
            np.array([179, 255, 255])
        )
    ],

    # White: low saturation and high brightness.
    "White": [
        (
            np.array([0, 0, 140]),
            np.array([179, 70, 255])
        )
    ],

    # Beige, tan, light brown and common wooden shades.
    "Wood": [
        (
            np.array([5, 20, 35]),
            np.array([30, 220, 245])
        )
    ]
}


DRAW_COLORS = {
    "Red": (0, 0, 255),
    "Orange": (0, 140, 255),
    "Yellow": (0, 255, 255),
    "Green": (0, 255, 0),
    "Cyan": (255, 255, 0),
    "Blue": (255, 0, 0),
    "Purple": (255, 0, 255),
    "Pink": (180, 105, 255),
    "White": (220, 220, 220),
    "Wood": (40, 130, 210),
    "Unknown": (255, 255, 255)
}


# =========================================================
# BACKGROUND CALIBRATION
# =========================================================

def capture_background(camera, number_of_frames=30):
    print()
    print("BACKGROUND CALIBRATION")
    print("Remove all objects from the camera view.")
    print("Keep the camera and background still.")
    print("Capturing in 3 seconds...")

    time.sleep(3)

    captured_frames = []

    for index in range(number_of_frames):
        ret, frame = camera.read()

        if not ret:
            continue

        blurred = cv2.GaussianBlur(
            frame,
            (7, 7),
            0
        )

        captured_frames.append(
            blurred.astype(np.float32)
        )

        print(
            f"Capturing background: {index + 1}/{number_of_frames}",
            end="\r"
        )

    print()

    if len(captured_frames) == 0:
        return None

    background = np.mean(
        captured_frames,
        axis=0
    ).astype(np.uint8)

    print("Background calibration complete.")
    print("You may now place objects in view.")

    return background


# =========================================================
# FOREGROUND DETECTION
# =========================================================

def create_foreground_mask(frame, background):
    blurred = cv2.GaussianBlur(
        frame,
        (7, 7),
        0
    )

    difference = cv2.absdiff(
        blurred,
        background
    )

    blue_difference, green_difference, red_difference = cv2.split(
        difference
    )

    strongest_difference = cv2.max(
        blue_difference,
        cv2.max(
            green_difference,
            red_difference
        )
    )

    _, mask = cv2.threshold(
        strongest_difference,
        BACKGROUND_THRESHOLD,
        255,
        cv2.THRESH_BINARY
    )

    opening_kernel = np.ones(
        (3, 3),
        np.uint8
    )

    closing_kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        opening_kernel,
        iterations=1
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        closing_kernel,
        iterations=1
    )

    return mask


# =========================================================
# COLOUR DETECTION
# =========================================================

def detect_colour(hsv_frame, object_mask):
    object_pixels = cv2.countNonZero(
        object_mask
    )

    if object_pixels == 0:
        return "Unknown", 0.0

    best_colour = "Unknown"
    best_ratio = 0.0

    for colour_name, ranges in COLOR_RANGES.items():
        combined_mask = np.zeros(
            hsv_frame.shape[:2],
            dtype=np.uint8
        )

        for lower, upper in ranges:
            current_mask = cv2.inRange(
                hsv_frame,
                lower,
                upper
            )

            combined_mask = cv2.bitwise_or(
                combined_mask,
                current_mask
            )

        colour_inside_object = cv2.bitwise_and(
            combined_mask,
            object_mask
        )

        matching_pixels = cv2.countNonZero(
            colour_inside_object
        )

        ratio = matching_pixels / object_pixels

        if ratio > best_ratio:
            best_ratio = ratio
            best_colour = colour_name

    # The object is still highlighted when colour is unknown.
    if best_ratio < 0.12:
        return "Unknown", best_ratio

    return best_colour, best_ratio


# =========================================================
# SHAPE CLASSIFICATION
# =========================================================

def classify_shape(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(
        contour,
        True
    )

    if area <= 0 or perimeter <= 0:
        return "Object"

    x, y, width, height = cv2.boundingRect(
        contour
    )

    aspect_ratio = width / float(height)

    circularity = (
        4.0 * math.pi * area
        / (perimeter * perimeter)
    )

    approximate = cv2.approxPolyDP(
        contour,
        0.035 * perimeter,
        True
    )

    vertices = len(approximate)

    if (
        circularity >= 0.74
        and 0.70 <= aspect_ratio <= 1.35
    ):
        return "Circular face"

    if vertices == 3:
        return "Triangular face"

    if vertices == 4:
        return "Quadrilateral face"

    if vertices == 5:
        return "Pentagonal face"

    if vertices == 6:
        return "Hexagonal face"

    return "Object"


# =========================================================
# CONTOUR FILTERING
# =========================================================

def get_valid_contours(mask, frame_area):
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    valid_contours = []

    maximum_area = (
        frame_area * MAX_OBJECT_AREA_RATIO
    )

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < MIN_OBJECT_AREA:
            continue

        if area > maximum_area:
            continue

        x, y, width, height = cv2.boundingRect(
            contour
        )

        if width < MIN_OBJECT_WIDTH:
            continue

        if height < MIN_OBJECT_HEIGHT:
            continue

        aspect_ratio = width / float(height)

        if aspect_ratio > 9.0:
            continue

        if aspect_ratio < 0.11:
            continue

        valid_contours.append(contour)

    return valid_contours


# =========================================================
# SCALE CALIBRATION
# =========================================================

def calibrate_scale(contours):
    global pixels_per_inch
    global measurement_samples
    global final_measurement
    global last_measurement_type

    if len(contours) == 0:
        print("Calibration failed: no object detected.")
        return

    reference_contour = max(
        contours,
        key=cv2.contourArea
    )

    rectangle = cv2.minAreaRect(
        reference_contour
    )

    side_one = rectangle[1][0]
    side_two = rectangle[1][1]

    reference_width_pixels = max(
        side_one,
        side_two
    )

    if reference_width_pixels <= 0:
        print("Calibration failed: invalid object size.")
        return

    pixels_per_inch = (
        reference_width_pixels
        / KNOWN_REFERENCE_WIDTH_INCHES
    )

    measurement_samples = []
    final_measurement = None
    last_measurement_type = None

    print()
    print("SCALE CALIBRATION COMPLETE")
    print(
        "Reference width:",
        KNOWN_REFERENCE_WIDTH_INCHES,
        "inches"
    )
    print(
        "Detected width:",
        round(reference_width_pixels, 1),
        "pixels"
    )
    print(
        "Pixels per inch:",
        round(pixels_per_inch, 2)
    )
    print("Measurement averaging reset.")


# =========================================================
# AVERAGING
# =========================================================

def update_measurement_average(current_measurement):
    global measurement_samples
    global final_measurement
    global last_measurement_type

    if current_measurement is None:
        return

    current_type = current_measurement["type"]

    # If shape type changes, begin a fresh set of 10 measurements.
    if (
        last_measurement_type is not None
        and current_type != last_measurement_type
        and final_measurement is None
    ):
        measurement_samples = []

    last_measurement_type = current_type

    if final_measurement is not None:
        return

    measurement_samples.append(
        current_measurement
    )

    print(
        f"Measurement sample "
        f"{len(measurement_samples)}/"
        f"{MEASUREMENTS_TO_AVERAGE}",
        end="\r"
    )

    if len(measurement_samples) < MEASUREMENTS_TO_AVERAGE:
        return

    valid_samples = [
        sample
        for sample in measurement_samples
        if sample["type"] == current_type
    ]

    if len(valid_samples) < MEASUREMENTS_TO_AVERAGE:
        measurement_samples = valid_samples
        return

    if current_type == "circle":
        average_diameter = sum(
            sample["diameter"]
            for sample in valid_samples
        ) / len(valid_samples)

        final_measurement = {
            "type": "circle",
            "diameter": average_diameter
        }

    else:
        average_length = sum(
            sample["length"]
            for sample in valid_samples
        ) / len(valid_samples)

        average_width = sum(
            sample["width"]
            for sample in valid_samples
        ) / len(valid_samples)

        final_measurement = {
            "type": "rectangle",
            "length": average_length,
            "width": average_width
        }

    print()
    print("FINAL AVERAGED MEASUREMENT")

    if final_measurement["type"] == "circle":
        print(
            "Diameter:",
            round(
                final_measurement["diameter"],
                2
            ),
            "inches"
        )
    else:
        print(
            "Length:",
            round(
                final_measurement["length"],
                2
            ),
            "inches"
        )

        print(
            "Width:",
            round(
                final_measurement["width"],
                2
            ),
            "inches"
        )


# =========================================================
# DRAWING AND MEASUREMENT
# =========================================================

def draw_detected_object(
    frame,
    contour,
    colour_name,
    shape_name,
    colour_confidence
):
    draw_colour = DRAW_COLORS.get(
        colour_name,
        DRAW_COLORS["Unknown"]
    )

    x, y, bounding_width, bounding_height = cv2.boundingRect(
        contour
    )

    current_measurement = None

    if shape_name == "Circular face":
        (center_x, center_y), radius_pixels = cv2.minEnclosingCircle(
            contour
        )

        center = (
            int(center_x),
            int(center_y)
        )

        radius_pixels = float(radius_pixels)

        cv2.circle(
            frame,
            center,
            int(radius_pixels),
            draw_colour,
            3
        )

        if pixels_per_inch is not None:
            diameter_inches = (
                2.0 * radius_pixels
                / pixels_per_inch
            )

            current_measurement = {
                "type": "circle",
                "diameter": diameter_inches
            }

    else:
        rectangle = cv2.minAreaRect(
            contour
        )

        box_points = cv2.boxPoints(
            rectangle
        )

        box_points = box_points.astype(
            np.int32
        )

        cv2.drawContours(
            frame,
            [box_points],
            0,
            draw_colour,
            3
        )

        side_one_pixels = rectangle[1][0]
        side_two_pixels = rectangle[1][1]

        visible_length_pixels = max(
            side_one_pixels,
            side_two_pixels
        )

        visible_width_pixels = min(
            side_one_pixels,
            side_two_pixels
        )

        if pixels_per_inch is not None:
            current_measurement = {
                "type": "rectangle",
                "length": (
                    visible_length_pixels
                    / pixels_per_inch
                ),
                "width": (
                    visible_width_pixels
                    / pixels_per_inch
                )
            }

    update_measurement_average(
        current_measurement
    )

    if pixels_per_inch is None:
        measurement_text = "Press C to calibrate inches"

    elif final_measurement is None:
        measurement_text = (
            f"Measuring: "
            f"{len(measurement_samples)}/"
            f"{MEASUREMENTS_TO_AVERAGE}"
        )

    elif final_measurement["type"] == "circle":
        measurement_text = (
            f"Final diameter: "
            f"{final_measurement['diameter']:.2f} in"
        )

    else:
        measurement_text = (
            f"Final L: "
            f"{final_measurement['length']:.2f} in  "
            f"W: "
            f"{final_measurement['width']:.2f} in"
        )

    label_y = max(
        y - 38,
        25
    )

    cv2.putText(
        frame,
        f"{colour_name} - {shape_name}",
        (x, label_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        draw_colour,
        2
    )

    cv2.putText(
        frame,
        measurement_text,
        (x, label_y + 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        draw_colour,
        2
    )

    if colour_name == "Unknown":
        colour_text = "Colour: unknown"
    else:
        colour_text = (
            f"Colour match: "
            f"{colour_confidence * 100:.0f}%"
        )

    text_y = min(
        y + bounding_height + 21,
        frame.shape[0] - 15
    )

    cv2.putText(
        frame,
        colour_text,
        (x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.44,
        draw_colour,
        1
    )


# =========================================================
# CAMERA
# =========================================================

camera = cv2.VideoCapture(
    CAMERA_DEVICE,
    cv2.CAP_V4L2
)

camera.set(
    cv2.CAP_PROP_FRAME_WIDTH,
    FRAME_WIDTH
)

camera.set(
    cv2.CAP_PROP_FRAME_HEIGHT,
    FRAME_HEIGHT
)

camera.set(
    cv2.CAP_PROP_FPS,
    30
)


if not camera.isOpened():
    print("ERROR: Raspberry Pi camera could not be opened.")
    print("Stop raspivid or ffplay before running this.")
    raise SystemExit


print("Raspberry Pi camera opened successfully.")

background = capture_background(
    camera
)

if background is None:
    print("ERROR: Background calibration failed.")
    camera.release()
    raise SystemExit


print()
print("CONTROLS")
print("B = recalibrate empty background")
print("C = calibrate inches using reference object")
print("R = reset the 10-reading measurement average")
print("Q = quit")
print()
print(
    "For C calibration, place only one reference object "
    "whose visible width is",
    KNOWN_REFERENCE_WIDTH_INCHES,
    "inches."
)


# =========================================================
# MAIN LOOP
# =========================================================

while True:
    ret, frame = camera.read()

    if not ret:
        print("ERROR: Could not read camera frame.")
        break

    foreground_mask = create_foreground_mask(
        frame,
        background
    )

    frame_area = (
        frame.shape[0]
        * frame.shape[1]
    )

    valid_contours = get_valid_contours(
        foreground_mask,
        frame_area
    )

    # Largest object first.
    valid_contours.sort(
        key=cv2.contourArea,
        reverse=True
    )

    hsv_frame = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    object_count = 0

    for contour in valid_contours:
        object_mask = np.zeros(
            frame.shape[:2],
            dtype=np.uint8
        )

        cv2.drawContours(
            object_mask,
            [contour],
            -1,
            255,
            thickness=-1
        )

        colour_name, colour_confidence = detect_colour(
            hsv_frame,
            object_mask
        )

        shape_name = classify_shape(
            contour
        )

        draw_detected_object(
            frame,
            contour,
            colour_name,
            shape_name,
            colour_confidence
        )

        object_count += 1

        # Average one object at a time.
        # The largest detected object is used.
        break

    if object_count == 0:
        status_text = "Looking for one object..."
        status_colour = (0, 0, 255)
    else:
        status_text = "Object detected"
        status_colour = (0, 255, 0)

    cv2.putText(
        frame,
        status_text,
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        status_colour,
        2
    )

    if pixels_per_inch is None:
        scale_text = "Scale: not calibrated"
    else:
        scale_text = (
            f"Scale calibrated: "
            f"{pixels_per_inch:.1f} px/in"
        )

    cv2.putText(
        frame,
        scale_text,
        (15, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1
    )

    cv2.putText(
        frame,
        "B: background  C: inches  R: reset  Q: quit",
        (15, frame.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1
    )

    cv2.imshow(
        "AGV Object Detection and Averaged Dimensions",
        frame
    )

    cv2.imshow(
        "Foreground Mask",
        foreground_mask
    )

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

    if key == ord("b"):
        new_background = capture_background(
            camera
        )

        if new_background is not None:
            background = new_background
            pixels_per_inch = None
            measurement_samples = []
            final_measurement = None
            last_measurement_type = None
        else:
            print("Background recalibration failed.")

    if key == ord("c"):
        calibrate_scale(
            valid_contours
        )

    if key == ord("r"):
        measurement_samples = []
        final_measurement = None
        last_measurement_type = None

        print()
        print("Measurement averaging reset.")
        print("The next 10 valid readings will be averaged.")


camera.release()
cv2.destroyAllWindows()

print("Camera stopped.")
