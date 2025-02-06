import re
import matplotlib.pyplot as plt
import argparse

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

def time_based_result(file_paths, labels, graph_name):
    colors = ['green', 'blue', 'red', 'purple', 'orange', 'cyan']
    markers = ['o', 's', 'x', '^', 'd', '*']
    
    plt.figure(figsize=(12, 6))
    
    # **Graph 1: Correct Detections**
    plt.subplot(2, 1, 1)
    for idx, (file_path, label) in enumerate(zip(file_paths, labels)):
        results = parse_detection_log(file_path)
        timestamps = []
        correct_detections = []
        
        for parsed_entry in results:
            correct_count = sum(1 for detected_obj in parsed_entry["detected_objects"]
                                if any(is_inside(detected_obj, model) for model in parsed_entry["filtered_models"]))
            
            timestamps.append(parsed_entry["timestamp"])
            correct_detections.append(correct_count)
        
        plt.plot(timestamps, correct_detections, label=f"{label} - Correct", marker=markers[idx % len(markers)], color=colors[idx % len(colors)])
    
    plt.ylabel("Correct Detections")
    plt.title(f"{graph_name}: Correct Detections Over Time")
    plt.legend()
    plt.grid(True)

    # **Graph 2: False Detections**
    plt.subplot(2, 1, 2)
    for idx, (file_path, label) in enumerate(zip(file_paths, labels)):
        results = parse_detection_log(file_path)
        timestamps = []
        false_detections = []
        
        for parsed_entry in results:
            false_count = sum(1 for detected_obj in parsed_entry["detected_objects"]
                              if not any(is_inside(detected_obj, model) for model in parsed_entry["filtered_models"]))
            
            timestamps.append(parsed_entry["timestamp"])
            false_detections.append(false_count)
        
        plt.plot(timestamps, false_detections, label=f"{label} - False", linestyle='dashed', marker=markers[idx % len(markers)], color=colors[idx % len(colors)])
    
    plt.xlabel("Time (s)")
    plt.ylabel("False Detections")
    plt.title(f"{graph_name}: False Detections Over Time")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate object detection performance over time.")
    parser.add_argument("file_paths", nargs='+', help="List of detection log files to compare")
    parser.add_argument("--labels", nargs='+', required=True, help="Labels for each detection log file")
    parser.add_argument("--graph_name", type=str, default="Detection Performance", help="Title for the graph")
    args = parser.parse_args()
    
    if len(args.file_paths) != len(args.labels):
        raise ValueError("Number of file paths must match the number of labels")
    
    time_based_result(args.file_paths, args.labels, args.graph_name)

