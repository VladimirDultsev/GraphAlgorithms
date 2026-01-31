import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_results_from_csv(filepath):
    try:
        df = pd.read_csv(filepath, comment='#')
        if 'Swaps' not in df.columns:
            print("Error: swaps column not found in CSV")
            return

        algorithms = [col for col in df.columns if col != 'Swaps']

        if not algorithms:
            print("No algorithms found")
            return

        df['Swaps'] = pd.to_numeric(df['Swaps'], errors='coerce')
        df = df.dropna(subset=['Swaps'])
        df['Swaps'] = df['Swaps'].astype(int)

        data = {}
        for algo in algorithms:
            data[algo] = df[algo].tolist()

        threshold_seconds = 15

        converted_data = {}
        for algorithm in data:
            converted = []
            for value in data[algorithm]:
                if value == 0:
                    converted.append(threshold_seconds)
                else:
                    converted.append(value / 1e6)
            converted_data[algorithm] = converted

        steps = df['Swaps'].tolist()
        plt.figure(figsize=(16, 10))
        styles = {
            "BFS": {"color": "blue", "marker": "o", "linewidth": 1.5, "alpha": 0.7},
            "2BFS": {"color": "green", "marker": "s", "linewidth": 1.5, "alpha": 0.7},
            "Multithread_2BFS": {"color": "lime", "marker": "D", "linewidth": 2.5, "markersize": 6, "alpha": 0.9},
            "A*": {"color": "red", "marker": "^", "linewidth": 1.5, "alpha": 0.7},
            "2A*": {"color": "purple", "marker": "v", "linewidth": 1.5, "alpha": 0.7},
            "Multithread_2A*": {"color": "magenta", "marker": "X", "linewidth": 2.5, "markersize": 6, "alpha": 0.9}
        }
        for algorithm in converted_data:
            style = styles.get(algorithm, {})
            plt.plot(steps, converted_data[algorithm],
                     marker=style.get("marker", "o"),
                     markersize=style.get("markersize", 4),
                     label=algorithm,
                     color=style.get("color", "black"),
                     linewidth=style.get("linewidth", 1.5),
                     alpha=style.get("alpha", 0.8))

        plt.axhline(y=threshold_seconds, color='r', linestyle='--', linewidth=2,
                    label=f'Порог ({threshold_seconds} сек)', alpha=0.7)
        plt.xlabel('Количество перестановок в оптимальном решении', fontsize=12)
        plt.ylabel('Время (секунды)', fontsize=12)
        plt.title('Сравнение времени работы алгоритмов поиска оптимального решения пятнашек', fontsize=14)
        plt.legend(fontsize=10, loc='upper left')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error processing file: {e}")
        import traceback
        traceback.print_exc()

def main():
    filepath = input("Enter CSV file path: ").strip()
    if os.path.exists(filepath):
        plot_results_from_csv(filepath)
    else:
        print(f"File not found: {filepath}")

if __name__ == "__main__":
    main()