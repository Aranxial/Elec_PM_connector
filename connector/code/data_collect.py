import subprocess

def get_model_names():
    """Get all model names from Ignition Gazebo."""
    result = subprocess.run(['ign', 'model', '--list'], stdout=subprocess.PIPE, text=True)
    models = []
    lines = result.stdout.splitlines()
    for line in lines:
        if line.startswith("    - "):
            model_name = line.split('- ')[1].strip()
            models.append(model_name)
    return models

def filter_connect_models(models):
    """Filter models that start with 'connect_'."""
    return [model for model in models if model.startswith('connect_')]

def get_model_pose(model_name):
    """Get pose information for a specific model."""
    result = subprocess.run(['ign', 'model', '-m', model_name], stdout=subprocess.PIPE, text=True)
    return result.stdout

def extract_xyz(pose_info):
    """Extract XYZ coordinates from the pose information."""
    lines = pose_info.splitlines()
    xyz = None
    for i, line in enumerate(lines):
        if "Pose [ XYZ (m) ] [ RPY (rad) ]:" in line:
            xyz_line = lines[i+1].strip()
            xyz_values = xyz_line.strip('[]').split()
            xyz = {
                'x': xyz_values[0],
                'y': xyz_values[1],
                'z': xyz_values[2]
            }
            break
    return xyz

def main():
    models = get_model_names()
    connect_models = filter_connect_models(models)
    
    for model in connect_models:
        pose_info = get_model_pose(model)
        xyz = extract_xyz(pose_info)
        if xyz:
            print(f"Model: {model}")
            print(f"Position - X: {xyz['x']}, Y: {xyz['y']}, Z: {xyz['z']}\n")
        else:
            print(f"Could not retrieve XYZ data for model: {model}")

if __name__ == "__main__":
    main()

