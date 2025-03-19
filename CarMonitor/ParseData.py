
# 8,0,13,88;59,0,68,83;8,0,13,88;59,0,68,83;-17.600,1.000,140.800;-9.222,1.000,544.111;-12.110,1.000,405.056;39.500,0.000;36.484,36.794;0.035;1.000;1.321;38.426;2.000;0;32,20,22,21;0,0,0,0;1;16.000;43.436;0.000;28.826;0.000;0.000;0.000;0.000;0.000;0.000;0.000;0.000

def process_raw_data(raw_data):
    # Split the raw_data string by semicolons
    fff = "ciao"

    if len(raw_data) <= 0:
        return {}
    raw_data = raw_data.split(";")

    # Function to split and convert to float
    def convert_to_float(data_str):
        return [float(x) for x in data_str.split(",")]

    def convert_to_float_tuple(data_str):
        return (float(x) for x in data_str.split(","))

    def parse_vector(data_str):
        splitted_data = data_str.split(",")
        return [(float(splitted_data[0]), float(splitted_data[1])), (float(splitted_data[2]), float(splitted_data[3]))]

    # Create a dictionary to hold the processed data
    data_dict = {
        'leftVectorOld': parse_vector(raw_data[0]),
        'rightVectorOld': parse_vector(raw_data[1]),
        'leftVector': parse_vector(raw_data[2]),
        'rightVector': parse_vector(raw_data[3]),
        'leftLine': convert_to_float_tuple(raw_data[4]),
        'rightLine': convert_to_float_tuple(raw_data[5]),
        'middleLaneLine': convert_to_float_tuple(raw_data[6]),
        'carPosition': convert_to_float(raw_data[7]),
        'newWayPointPosition': convert_to_float(raw_data[8]),
        'finish_line_left_segment': parse_vector(raw_data[15]),
        'finish_line_right_segment': parse_vector(raw_data[16]),

        # Scalar values
        'steeringWheelAngle': float(raw_data[9]),
        'carAcceleration': float(raw_data[10]),
        'frontObstacleDistance': float(raw_data[11]),
        'lookAheadDistance': float(raw_data[12]),
        'carSpeedRaw': float(raw_data[13]),
        'g_finish_line_detected': float(raw_data[14]),
        'g_finish_line_detected_now': float(raw_data[17]),
        'g_loop_time_ms': float(raw_data[18]),

        # Wheel RPM values
        'left_wheel_raw_rpm': float(raw_data[19]),
        'left_wheel_adjusted_rpm': float(raw_data[20]),
        'right_wheel_raw_rpm': float(raw_data[21]),
        'right_wheel_adjusted_rpm': float(raw_data[22]),

        # Wheel speed request values
        'left_wheel_speed_request_raw': float(raw_data[23]),
        'right_wheel_speed_request_raw': float(raw_data[24])
    }

    return data_dict
