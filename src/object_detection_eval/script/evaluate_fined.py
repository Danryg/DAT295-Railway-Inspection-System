import re
import matplotlib.pyplot as plt
import argparse
from collections import defaultdict

def is_inside(detected_obj, filtered_model):
    obj_x, obj_y, obj_z = detected_obj["center"]
    obj_w, obj_h, obj_d = detected_obj["size"]
    
    model_x, model_y, model_z = filtered_model["center"]
    model_w, model_h, model_d = (filtered_model["width"], 
                                 filtered_model["height"], 
                                 filtered_model["depth"])
    
    # NOTE: Check carefully if the second dimension in your y-check
    # should use model_w or model_d. Example below uses model_d for y.
    return (
        (model_x - model_w / 2 <= obj_x <= model_x + model_w / 2) and
        (model_y - model_d / 2 <= obj_y <= model_y + model_d / 2) and
        (model_z - model_h / 2 <= obj_z <= model_z + model_h / 2)
    )

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
        
        detected_objects = re.findall(
            r'Center: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)\s+Size: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)',
            entry
        )
        for obj in detected_objects:
            parsed_entry["detected_objects"].append({
                "center": tuple(map(float, obj[:3])),
                "size": tuple(map(float, obj[3:]))
            })
        
        filtered_models = re.findall(
            r'Model: ([^,]+), Distance: ([-\d\.]+), Width: ([-\d\.]+), Height: ([-\d\.]+), Depth: ([-\d\.]+)\s+Center: \(([-\d\.]+), ([-\d\.]+), ([-\d\.]+)\)',
            entry
        )
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
    
    # ======================================================
    # 1) Correct Detections (Top Plot)
    # ======================================================
    plt.subplot(2, 1, 1)
    for idx, (file_path, label) in enumerate(zip(file_paths, labels)):
        results = parse_detection_log(file_path)
        
        # Group results by integer-second bin
        correct_by_second = defaultdict(lambda: {"sum": 0, "count": 0})
        
        for parsed_entry in results:
            second_bin = int(parsed_entry["timestamp"])
            
            # Instead of sum(...) with a generator, use a small loop:
            correct_count = 0
            for detected_obj in parsed_entry["detected_objects"]:
                # Check if it is inside ANY filtered_model
                inside_any = False
                for model in parsed_entry["filtered_models"]:
                    if is_inside(detected_obj, model):
                        inside_any = True
                        break
                if inside_any:
                    correct_count += 1
            
            correct_by_second[second_bin]["sum"] += correct_count
            correct_by_second[second_bin]["count"] += 1
        
        # Now compute the average correct detections in each second
        bins_sorted = sorted(correct_by_second.keys())
        timestamps = []
        avg_correct_detections = []
        
        for second_bin in bins_sorted:
            total = correct_by_second[second_bin]["sum"]
            cnt = correct_by_second[second_bin]["count"]
            avg = total / cnt  # average correct detections per entry in that second
            timestamps.append(second_bin)
            avg_correct_detections.append(avg)
        
        plt.plot(
            timestamps,
            avg_correct_detections,
            label=f"{label} - Correct",
            marker=markers[idx % len(markers)],
            color=colors[idx % len(colors)]
        )
    
    plt.ylabel("Avg Correct Detections")
    plt.title(f"{graph_name}: Correct Detections Over Time (1-sec bins)")
    plt.legend()
    plt.grid(True)

    # ======================================================
    # 2) False Detections (Bottom Plot)
    # ======================================================
    plt.subplot(2, 1, 2)
    for idx, (file_path, label) in enumerate(zip(file_paths, labels)):
        results = parse_detection_log(file_path)
        
        false_by_second = defaultdict(lambda: {"sum": 0, "count": 0})
        
        for parsed_entry in results:
            second_bin = int(parsed_entry["timestamp"])
            
            # Similarly, expand false_count logic
            false_count = 0
            for detected_obj in parsed_entry["detected_objects"]:
                inside_any = False
                for model in parsed_entry["filtered_models"]:
                    if is_inside(detected_obj, model):
                        inside_any = True
                        break
                if not inside_any:
                    false_count += 1
            
            false_by_second[second_bin]["sum"] += false_count
            false_by_second[second_bin]["count"] += 1
        
        # Compute average false detections in each second
        bins_sorted = sorted(false_by_second.keys())
        timestamps = []
        avg_false_detections = []
        
        for second_bin in bins_sorted:
            total = false_by_second[second_bin]["sum"]
            cnt = false_by_second[second_bin]["count"]
            avg = total / cnt
            timestamps.append(second_bin)
            avg_false_detections.append(avg)
        
        plt.plot(
            timestamps,
            avg_false_detections,
            label=f"{label} - False",
            linestyle='dashed',
            marker=markers[idx % len(markers)],
            color=colors[idx % len(colors)]
        )
    
    plt.xlabel("Time (s)")
    plt.ylabel("Avg False Detections")
    plt.title(f"{graph_name}: False Detections Over Time (1-sec bins)")
    plt.legend()
    plt.grid(True)
    
    plt.suptitle("Graph by Keyvan", fontsize=14)
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

