import os
import pandas as pd
import matplotlib.pyplot as plt

def list_csv_files():
    files = [f for f in os.listdir('.') if f.endswith('.csv')]
    if not files:
        print("No CSV files found.")
        return []
    
    print("\nAvailable CSV files:")
    for i, f in enumerate(files):
        print(f"{i + 1}: {f}")
    return files

def choose_file(files):
    while True:
        try:
            index = int(input("\nEnter the number of the file to visualize: ")) - 1
            if 0 <= index < len(files):
                return files[index]
            else:
                print("Invalid selection.")
        except ValueError:
            print("Enter a valid number.")

def plot_and_compare(df, file_name):
    df['time'] = df['time'] - df['time'].iloc[0]

    plt.figure(figsize=(12, 10))

    # Linear X
    plt.subplot(3, 1, 1)
    plt.plot(df['time'], df['cmd_linear_x'], '--', label='cmd_linear_x')
    plt.plot(df['time'], df['odom_linear_x'], label='odom_linear_x')
    plt.ylabel('Linear X (m/s)')
    plt.title('Commanded vs Actual Velocity')
    plt.legend()
    plt.grid(True)

    # Linear Y
    plt.subplot(3, 1, 2)
    plt.plot(df['time'], df['cmd_linear_y'], '--', label='cmd_linear_y')
    plt.plot(df['time'], df['odom_linear_y'], label='odom_linear_y')
    plt.ylabel('Linear Y (m/s)')
    plt.legend()
    plt.grid(True)

    # Angular Z
    plt.subplot(3, 1, 3)
    plt.plot(df['time'], df['cmd_angular_z'], '--', label='cmd_angular_z')
    plt.plot(df['time'], df['odom_angular_z'], label='odom_angular_z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Z (rad/s)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()

    return plt

def compute_errors(df):
    print("\n--- Error Summary ---")
    for axis in ['linear_x', 'linear_y', 'angular_z']:
        cmd = df[f'cmd_{axis}']
        odom = df[f'odom_{axis}']
        abs_error = (cmd - odom).abs()
        mae = abs_error.mean()
        std = abs_error.std()
        print(f"{axis}: MAE = {mae:.4f}, Std Dev = {std:.4f}")

def main():
    files = list_csv_files()
    if not files:
        return

    selected_file = choose_file(files)
    df = pd.read_csv(selected_file)

    # Check required columns
    required = [
        'time',
        'cmd_linear_x', 'cmd_linear_y', 'cmd_angular_z',
        'odom_linear_x', 'odom_linear_y', 'odom_angular_z'
    ]
    if not all(col in df.columns for col in required):
        print(f"Missing expected columns in {selected_file}.")
        return

    compute_errors(df)
    plot = plot_and_compare(df, selected_file)

    choice = input("\nDo you want to save the plot? [y/n]: ").strip().lower()
    if choice == 'y':
        img_name = selected_file.replace('.csv', '.png')
        plot.savefig(img_name)
        print(f"Saved plot as: {img_name}")
    else:
        plot.show()

if __name__ == '__main__':
    main()
