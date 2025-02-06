import re
import matplotlib.pyplot as plt

def is_inside(detected_obj, filtered_model):
    obj_x, obj_y, obj_z = detected_obj["center"]
    obj_w, obj_h, obj_d = detected_obj["size"]
    
    model_x, model_y, model_z = filtered_model["center"]
    model_w, model_h, model_d = filtered_model["width"], filtered_model["height"], filtered_model["depth"]
    
    return (model_x - model_w / 2 <= obj_x <= model_x + model_w / 2 and
            model_y - model_d / 2 <= obj_y <= model_y + model_w / 2 and
            model_z - model_h / 2 <= obj_z <= model_z + model_h / 2)

def parse_detection_log(file_path):
    results = []
    
    with open(file_path, 'r') as file:
        content = file.read()
    
    entries = content.split("====================================")
    
    for entry in entries:
        entry = entry.strip()
        if not entry:
            continue
        
        parsed_entry = {
            "timestamp": None,
            "detected_objects": [],
            "filtered_models": []
        }
        
        timestamp_match = re.search(r'Timestamp: ([\d\.]+)', entry)
        if timestamp_match:
            parsed_entry["timestamp"] = float(timestamp_match.group(1))
        
        detected_objects = re.findall(r'Center: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)\n  Size: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)', entry)
        for obj in detected_objects:
            parsed_entry["detected_objects"].append({
                "center": tuple(map(float, obj[:3])),
                "size": tuple(map(float, obj[3:]))
            })
        
        filtered_models = re.findall(r'Model: ([^,]+), Distance: ([-\d\.]+), Width: ([-\d\.]+), Height: ([-\d\.]+), Depth: ([-\d\.]+)\n  Center: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)', entry)
        for model in filtered_models:
            parsed_entry["filtered_models"].append({
                "name": model[0],
                "distance": float(model[1]),
                "width": float(model[2]),
                "height": float(model[3]),
                "depth": float(model[4]),
                "center": tuple(map(float, model[5:]))
            })
        
        results.append(parsed_entry)
    
    return results

def time_based_result(results):
    timestamps = []
    correct_detections = []
    false_detections = []
    
    for parsed_entry in results:
        false_count = 0
        correct_count = 0
        
        for detected_obj in parsed_entry["detected_objects"]:
            match_found = False
            for filtered_model in parsed_entry["filtered_models"]:
                if is_inside(detected_obj, filtered_model):
                    correct_count += 1
                    match_found = True
                    break
            if not match_found:
                false_count += 1
        
        timestamps.append(parsed_entry["timestamp"])
        correct_detections.append(correct_count)
        false_detections.append(false_count)
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    axes[0].plot(timestamps, correct_detections, label="Correct Detections", marker='o', color='green')
    axes[0].set_ylabel("Count")
    axes[0].set_title("Correct Detections Over Time")
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(timestamps, false_detections, label="False Detections", marker='x', color='red')
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Count")
    axes[1].set_title("False Detections Over Time")
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage
file_path = "/home/sod/chalmers/DAT295/DAT295-Railway-Inspection-System/output_data.txt"
parsed_data = parse_detection_log(file_path)
time_based_result(parsed_data)

