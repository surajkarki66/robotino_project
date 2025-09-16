import math
import requests
from PIL import Image
from io import BytesIO
import matplotlib.pyplot as plt
import numpy as np


class OpenRobotinoAPI:
    def __init__(self, base_url, api_token=None):
        """
        Initializes the API class.

        :param base_url: Base URL of the Robotino server (e.g., "http://192.168.0.1")
        :param api_token: Optional API token if authentication is required
        """
        self.base_url = base_url.rstrip("/")
        self.headers = {
            "Authorization": f"Bearer {api_token}" if api_token else None
        }

    def _get(self, endpoint):
        """
        Executes a GET request to a specified endpoint.

        :param endpoint: API endpoint (path relative to the base URL)
        :return: Response object
        """
        url = f"{self.base_url}/{endpoint.lstrip('/')}"
        try:
            response = requests.get(url, headers=self.headers)
            response.raise_for_status()
            return response
        except requests.RequestException as e:
            print(f"Error during request to {url}: {e}")
            return None

    def get_cam0_image(self):
        """
        Retrieves the camera image from cam0 and displays it.
        """
        response = self._get("cam0")
        if response:
            image_data = BytesIO(response.content)
            image = Image.open(image_data)
            image.show()
        else:
            print("Could not retrieve camera image from cam0.")

    def get_sensor_image(self):
        """
        Retrieves the sensor data image and displays it.
        """
        response = self._get("sensorimage")
        if response:
            image_data = BytesIO(response.content)
            image = Image.open(image_data)
            image.show()
        else:
            print("Could not retrieve sensor image.")

    def get_festool_charger_data(self):
        """
        Retrieves data of the Festool Li-Ion batteries.

        :return: Dictionary with battery data or None
        """
        response = self._get("data/festoolcharger")
        if response:
            return response.json()
        else:
            print("Could not retrieve Festool battery data.")
            return None

    def get_power_management_data(self):
        """
        Retrieves information about voltage and system current.

        :return: Dictionary with power management data or None
        """
        response = self._get("data/powermanagement")
        if response:
            return response.json()
        else:
            print("Could not retrieve power management data.")
            return None

    def get_charger_data(self, charger_id):
        """
        Retrieves data of the built-in charger.

        :param charger_id: ID of the charger (0 or 1)
        :return: Dictionary with charger data or None
        """
        response = self._get(f"data/charger{charger_id}")
        if response:
            return response.json()
        else:
            print(f"Could not retrieve data for charger {charger_id}.")
            return None

    def get_controller_info(self):
        """
        Retrieves information about hardware and software version.

        :return: Dictionary with controller information or None
        """
        response = self._get("data/controllerinfo")
        if response:
            return response.json()
        else:
            print("Could not retrieve controller information.")
            return None

    def get_services(self):
        """
        Retrieves data of Robotino-specific services.

        :return: Dictionary with service data or None
        """
        response = self._get("data/services")
        if response:
            return response.json()
        else:
            print("Could not retrieve service data.")
            return None

    def get_service_status(self, service_name):
        """
        Retrieves the status of a specific service.

        :param service_name: Name of the service
        :return: Dictionary with service status or None
        """
        response = self._get(f"data/servicestatus/{service_name}")
        if response:
            return response.json()
        else:
            print(f"Could not retrieve status for service {service_name}.")
            return None

    def get_analog_input_array(self):
        """
        Retrieves data of all analog inputs.

        :return: List with analog input values or None
        """
        response = self._get("data/analoginputarray")
        if response:
            return response.json()
        else:
            print("Could not retrieve analog input data.")
            return None

    def get_digital_input_array(self):
        """
        Retrieves data of all digital inputs.

        :return: List with digital input values (True/False) or None
        """
        response = self._get("data/digitalinputarray")
        if response:
            return response.json()
        else:
            print("Could not retrieve digital input data.")
            return None

    def get_digital_output_status(self):
        """
        Retrieves the status of all digital outputs.

        :return: List with digital output values (True/False) or None
        """
        response = self._get("data/digitaloutputstatus")
        if response:
            return response.json()
        else:
            print("Could not retrieve digital output status.")
            return None

    def get_relay_status(self):
        """
        Retrieves the status of all relays.

        :return: List with relay values (True/False) or None
        """
        response = self._get("data/relaystatus")
        if response:
            return response.json()
        else:
            print("Could not retrieve relay status.")
            return None

    def get_bumper_status(self):
        """
        Retrieves the status of the Robotino bumper.

        :return: Dictionary with bumper status or None
        """
        response = self._get("data/bumper")
        if response:
            return response.json()
        else:
            print("Could not retrieve bumper status.")
            return None

    def get_distance_sensor_array(self):
        """
        Retrieves the measurements of the distance sensors.

        :return: List with distance values or None
        """
        response = self._get("data/distancesensorarray")
        if response:
            return response.json()
        else:
            print("Could not retrieve distance sensor data.")
            return None

    def get_scan0_data(self):
        """
        Retrieves data from the first laser scanner and calculates angles and distances.

        :return: Dictionary with angles and distances or None
        {
            "angles": [List of angles],
            "distances": [List of distances]
        }
        """
        response = self._get("data/scan0")

        if response:
            data = response.json()
            print(f"raw: {data}")
            angle_min = data.get("angle_min")
            angle_max = data.get("angle_max")
            angle_increment = data.get("angle_increment")
            ranges = data.get("ranges")

            if angle_min is not None and angle_max is not None and angle_increment is not None and ranges:
                # Calculate angles
                num_angles = len(ranges)
                angles = [angle_min + i * angle_increment for i in range(num_angles)]
                angles = [angle + math.pi / 2 for angle in angles]
                print(f"\tangles: {angles}\n\tdistances: {ranges}")
                return {"angles": angles, "distances": ranges}
        print("Could not retrieve laser scan data.")
        return None

    def get_odometry(self):
        """
        Retrieves the odometry data of the robot.

        :return: List with odometry data or None
        """
        response = self._get("data/odometry")
        if response:
            return response.json()
        else:
            print("Could not retrieve odometry data.")
            return None

    def get_image_version(self):
        """
        Retrieves the version of the Robotino OS image.

        :return: Dictionary with image version or None
        """
        response = self._get("data/imageversion")
        if response:
            return response.json()
        else:
            print("Could not retrieve OS image version.")
            return None

    def get_power_output_current(self):
        """
        Retrieves the current of the Robotino power output.

        :return: Dictionary with current value or None
        """
        response = self._get("data/poweroutputcurrent")
        if response:
            return response.json()
        else:
            print("Could not retrieve power output current value.")
            return None

    def get_reflector_points(self, intensity_threshold=1):
        """
        Retrieves points from the first laser scanner that likely correspond to reflectors,
        based on a given intensity threshold.

        :param intensity_threshold: Minimum intensity value to consider a point as a reflector
        :return: List of tuples (angle, distance, intensity) for reflector points
        """
        response = self._get("data/scan0")
        if response:
            data = response.json()
            angle_min = data.get("angle_min")
            angle_increment = data.get("angle_increment")
            ranges = data.get("ranges")
            intensities = data.get("intensities")
            print(intensities)
            if angle_min is not None and angle_increment is not None and ranges and intensities:
                reflector_points = []
                for i, (distance, intensity) in enumerate(zip(ranges, intensities)):
                    if intensity >= intensity_threshold:
                        angle = angle_min + i * angle_increment
                        angle += math.pi / 2  # Apply the same rotation as in get_scan0_data
                        reflector_points.append((angle, distance, intensity))
                return reflector_points
        print("Could not retrieve or parse scan0 data for reflector extraction.")
        return []


# Example usage:
if __name__ == "__main__":
    # Base URL and optional token
    base_url = "http://172.21.21.90"
    api_token = None  # Optional, if authentication is needed

    # Create API client
    robotino23 = OpenRobotinoAPI(base_url, api_token)
    

    # Example: Retrieve and display sensor image
    #robotino23.get_sensor_image()

    # Example: Retrieve and display camera image from cam0
    # data = robotino23.get_distance_sensor_array()
    # print(data)

    # data = robotino23.get_scan0_data()
    # print(data)

    # Live plot for laser scan data
    plt.ion()
    fig, ax = plt.subplots()
    scan_plot, = ax.plot([], [], 'bo', markersize=2)

    plt.show()  # Ensure the plot window is displayed

    while True:
        scan_data = robotino23.get_scan0_data()
        if scan_data:
            angles = scan_data["angles"]
            distances = scan_data["distances"]

            if angles and distances:
                x_coords = [d * np.cos(a) for d, a in zip(distances, angles)]
                y_coords = [d * np.sin(a) for d, a in zip(distances, angles)]
                print(f"new data to draw\n\tx_coords: {x_coords}\n\ty_coords: {y_coords}")

                # Get reflector points
                reflector_points = robotino23.get_reflector_points(intensity_threshold=1)
                if reflector_points:
                    ref_x = [d * np.cos(a) for a, d, _ in reflector_points]
                    ref_y = [d * np.sin(a) for a, d, _ in reflector_points]
                else:
                    ref_x, ref_y = [], []

                ax.clear()
                ax.plot(x_coords, y_coords, 'bo', markersize=2)
                if ref_x and ref_y:
                    ax.plot(ref_x, ref_y, 'yo', markersize=6, label='Reflectors')
                # Draw robot at origin with an arrow indicating forward direction
                ax.plot(0, 0, marker=(3, 0, 90), color='red', markersize=15, label='Robot')
                ax.set_title("Laser scan data with reflectors and robot orientation")
                ax.set_xlabel("X-coordinates (m)")
                ax.set_ylabel("Y-coordinates (m)")
                ax.axis("equal")
                ax.set_xlim(-max(distances), max(distances))
                ax.set_ylim(-max(distances), max(distances))
                handles, labels = ax.get_legend_handles_labels()
                if ref_x and ref_y:
                    ax.legend()
                else:
                    # Show legend for robot only if no reflectors
                    ax.legend([handles[-1]], [labels[-1]])

                plt.pause(0.1)  # Update every 1 second
        #time.sleep(10)