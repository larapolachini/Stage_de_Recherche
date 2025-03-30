#!/usr/bin/env python3
# CMA-ES Parameter Tuning for Message Reception Probability Model

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import seaborn as sns
from scipy.optimize import minimize
import cma
import io

# Define the data as a string (replace this with loading from a file if needed)
csv_data = """payload_size,cluster_size,p_send,success_probability,std_deviation
3,18,0.1,0.998,0.0040
3,18,0.25,0.996,0.0112
3,18,0.5,0.993,0.0141
3,18,0.75,0.986,0.0179
3,18,0.9,0.967,0.0468
3,18,1,0.842,0.0999
3,4,0.1,0.998,0.0036
3,4,0.25,0.998,0.0036
3,4,0.5,0.996,0.0067
3,4,0.75,0.989,0.0179
3,4,0.9,0.993,0.0095
3,4,1,0.985,0.0260
3,2,0.1,0.986,0.0177
3,2,0.25,1.000,0.0000
3,2,0.5,0.995,0.0102
3,2,0.75,0.999,0.0030
3,2,0.9,1.000,0.0000
3,2,1,0.998,0.0036
20,18,0.1,0.997,0.0054
20,18,0.25,0.997,0.0043
20,18,0.5,0.882,0.1128
20,18,0.75,0.766,0.2111
20,18,0.9,0.570,0.3081
20,18,1,0.481,0.3092
20,4,0.1,0.989,0.0153
20,4,0.25,0.993,0.0085
20,4,0.5,0.994,0.0086
20,4,0.75,0.967,0.0388
20,4,0.9,0.775,0.1987
20,4,1,0.800,0.2340
20,2,0.1,0.992,0.0129
20,2,0.25,0.991,0.0151
20,2,0.5,0.992,0.0137
20,2,0.75,0.996,0.0080
20,2,0.9,0.997,0.0046
20,2,1,0.987,0.0231
100,18,0.01,0.976,0.0387
100,18,0.1,0.821,0.1679
100,18,0.25,0.395,0.3114
100,18,0.5,0.038,0.1089
100,18,0.75,0.018,0.0703
100,18,0.9,0.018,0.0454
100,18,1,0.025,0.0771
100,4,0.01,0.920,0.0872
100,4,0.1,0.929,0.0494
100,4,0.25,0.574,0.2788
100,4,0.5,0.166,0.1971
100,4,0.75,0.037,0.0645
100,4,0.9,0.030,0.0545
100,4,1,0.052,0.1012
100,2,0.01,0.912,0.1433
100,2,0.1,0.884,0.1164
100,2,0.25,0.815,0.1561
100,2,0.5,0.692,0.1971
100,2,0.75,0.634,0.1962
100,2,0.9,0.189,0.1629
100,2,1,0.131,0.1483"""

# Load the data into a DataFrame
df = pd.read_csv(io.StringIO(csv_data))
print(f"Loaded {len(df)} data points")

# Extract the features and target
X = df[['payload_size', 'cluster_size', 'p_send']].values
y = df['success_probability'].values

# Define the message reception probability model
def model(params, payload, cluster, p_send):
    """
    Calculate the success probability using our model.
    
    params: [a, b, c, d] for the model:
        success_prob = 1 / (1 + (a * payload^b * p_send^c / cluster^d))
    
    Returns: Predicted success probability
    """
    a, b, c, d = params
    # Ensure all parameters are positive to maintain model structure
    a = max(1e-6, a)  # a should be positive but can be very small
    b = max(0, b)     # b should be non-negative
    c = max(0, c)     # c should be non-negative
    d = max(0, d)     # d should be non-negative
    
    term = a * (payload ** b) * (p_send ** c) / (cluster ** d)
    return 1 / (1 + term)

# Define the loss function that CMA-ES will minimize
def loss_function(params):
    """
    Calculate the mean squared error between model predictions and actual values.
    """
    predictions = np.array([model(params, *xi) for xi in X])
    mse = np.mean((predictions - y) ** 2)
    return mse

# Define a function to calculate R-squared
def calculate_r_squared(params):
    """
    Calculate the R-squared value for the model.
    """
    predictions = np.array([model(params, *xi) for xi in X])
    ss_total = np.sum((y - np.mean(y)) ** 2)
    ss_residual = np.sum((y - predictions) ** 2)
    r_squared = 1 - (ss_residual / ss_total)
    return r_squared

# Define a function to analyze residuals by payload size
def analyze_residuals_by_payload(params):
    """
    Calculate mean absolute residuals grouped by payload size.
    """
    predictions = np.array([model(params, *xi) for xi in X])
    residuals = np.abs(y - predictions)
    
    # Add residuals to the DataFrame
    df_with_residuals = df.copy()
    df_with_residuals['residual'] = residuals
    
    # Group by payload size and calculate mean residual
    residuals_by_payload = df_with_residuals.groupby('payload_size')['residual'].mean()
    return residuals_by_payload

# Function to run CMA-ES optimization
def run_cma_es(initial_params, sigma=0.5, maxiter=1000):
    """
    Run the CMA-ES optimization algorithm to find the best parameters.
    
    initial_params: Starting parameter values [a, b, c, d]
    sigma: Initial step size
    maxiter: Maximum number of iterations
    
    Returns: Best parameters found
    """
    print(f"Starting CMA-ES optimization with initial parameters: {initial_params}")
    
    # Create CMA-ES optimizer
    es = cma.CMAEvolutionStrategy(initial_params, sigma)
    
    # Run optimization
    iteration = 0
    best_value = float('inf')
    history = []
    
    while not es.stop() and iteration < maxiter:
        # Ask for new parameter candidates
        solutions = es.ask()
        
        # Evaluate candidates
        values = [loss_function(sol) for sol in solutions]
        
        # Tell CMA-ES the results
        es.tell(solutions, values)
        
        # Update best found solution
        current_best = min(values)
        if current_best < best_value:
            best_value = current_best
            best_params = solutions[values.index(current_best)]
            print(f"Iteration {iteration}: MSE = {best_value:.6f}, Params = {best_params}")
            
        # Store history
        history.append(best_value)
        
        # Next iteration
        iteration += 1
    
    # Get final result
    result = es.result
    print(f"Optimization finished after {iteration} iterations")
    print(f"Best MSE: {best_value:.6f}")
    print(f"Best parameters: {result[0]}")
    
    # Plot convergence
    plt.figure(figsize=(10, 6))
    plt.plot(history)
    plt.title('CMA-ES Convergence')
    plt.xlabel('Iteration')
    plt.ylabel('Mean Squared Error')
    plt.yscale('log')
    plt.grid(True)
    plt.savefig('cma_es_convergence.png')
    plt.close()
    
    return result[0]

# Function to evaluate and visualize the model
def evaluate_model(params):
    """
    Evaluate the model with the given parameters and create visualizations.
    """
    # Calculate overall metrics
    predictions = np.array([model(params, *xi) for xi in X])
    mse = np.mean((predictions - y) ** 2)
    mae = np.mean(np.abs(predictions - y))
    max_error = np.max(np.abs(predictions - y))
    r_squared = calculate_r_squared(params)
    
    print("\nModel Evaluation:")
    print(f"Mean Squared Error: {mse:.6f}")
    print(f"Mean Absolute Error: {mae:.6f}")
    print(f"Maximum Absolute Error: {max_error:.6f}")
    print(f"R-squared: {r_squared:.6f}")
    
    # Analyze residuals by payload size
    residuals_by_payload = analyze_residuals_by_payload(params)
    print("\nMean absolute residuals by payload size:")
    for payload, residual in residuals_by_payload.items():
        print(f"Payload size {payload}: {residual:.6f}")
    
    # Create visualizations
    
    # 1. Actual vs Predicted scatter plot
    plt.figure(figsize=(10, 6))
    plt.scatter(y, predictions)
    plt.plot([0, 1], [0, 1], 'k--')
    plt.xlabel('Actual Success Probability')
    plt.ylabel('Predicted Success Probability')
    plt.title('Actual vs Predicted Success Probability')
    plt.grid(True)
    plt.savefig('actual_vs_predicted.png')
    plt.close()
    
    # 2. Residuals plot
    residuals = y - predictions
    plt.figure(figsize=(10, 6))
    plt.scatter(predictions, residuals)
    plt.axhline(y=0, color='k', linestyle='--')
    plt.xlabel('Predicted Success Probability')
    plt.ylabel('Residuals')
    plt.title('Residuals vs Predicted')
    plt.grid(True)
    plt.savefig('residuals.png')
    plt.close()
    
    # 3. Create heatmaps for different payload sizes
    payload_sizes = [3, 20, 100]
    
    for payload in payload_sizes:
        p_send_values = np.linspace(0, 1, 100)
        cluster_sizes = [2, 4, 18]
        
        # Create grid for the heatmap
        p_send_grid, cluster_grid = np.meshgrid(p_send_values, cluster_sizes)
        success_grid = np.zeros_like(p_send_grid, dtype=float)
        
        # Calculate model predictions for each grid point
        for i in range(len(cluster_sizes)):
            for j in range(len(p_send_values)):
                success_grid[i, j] = model(params, payload, cluster_sizes[i], p_send_values[j])
        
        # Create heatmap
        plt.figure(figsize=(12, 5))
        plt.imshow(success_grid, cmap='viridis', origin='lower', aspect='auto', 
                   extent=[0, 1, 0, len(cluster_sizes)-1])
        plt.colorbar(label='Success Probability')
        plt.xticks(np.linspace(0, 1, 11))
        plt.yticks(np.arange(len(cluster_sizes)), cluster_sizes)
        plt.xlabel('Send Probability (p_send)')
        plt.ylabel('Cluster Size')
        plt.title(f'Success Probability Heatmap (Payload Size = {payload})')
        plt.savefig(f'heatmap_payload_{payload}.png')
        plt.close()
    
    # 4. Create line plots for different configurations
    # Plot success probability vs send probability for different payload sizes
    plt.figure(figsize=(10, 6))
    for payload in payload_sizes:
        p_send_values = np.linspace(0, 1, 100)
        cluster_size = 4  # Fix cluster size
        
        success_probs = [model(params, payload, cluster_size, p) for p in p_send_values]
        plt.plot(p_send_values, success_probs, label=f'Payload {payload}')
    
    plt.xlabel('Send Probability')
    plt.ylabel('Success Probability')
    plt.title(f'Success Probability vs Send Probability (Cluster Size = {cluster_size})')
    plt.legend()
    plt.grid(True)
    plt.savefig('success_vs_psend_by_payload.png')
    plt.close()
    
    # Plot success probability vs send probability for different cluster sizes
    plt.figure(figsize=(10, 6))
    for cluster in [2, 4, 18]:
        p_send_values = np.linspace(0, 1, 100)
        payload_size = 20  # Fix payload size
        
        success_probs = [model(params, payload_size, cluster, p) for p in p_send_values]
        plt.plot(p_send_values, success_probs, label=f'Cluster {cluster}')
    
    plt.xlabel('Send Probability')
    plt.ylabel('Success Probability')
    plt.title(f'Success Probability vs Send Probability (Payload Size = {payload_size})')
    plt.legend()
    plt.grid(True)
    plt.savefig('success_vs_psend_by_cluster.png')
    plt.close()
    
    # Return the equation for reporting
    a, b, c, d = params
    equation = f"P_success = 1 / (1 + ({a:.6f} * payload_size^{b:.4f} * p_send^{c:.4f} / cluster_size^{d:.4f}))"
    return equation

# Main execution
if __name__ == "__main__":
    # Initial parameters based on our previous analysis
    initial_params = [0.001, 2.0, 1.5, 0.5]  # [a, b, c, d]
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Run CMA-ES optimization
    best_params = run_cma_es(initial_params, sigma=0.3, maxiter=500)
    
    # Evaluate the optimized model
    equation = evaluate_model(best_params)
    
    # Print the final model equation
    print("\nOptimized Model Equation:")
    print(equation)
    
    # Save the best parameters to a CSV file
    pd.DataFrame({
        'parameter': ['a', 'b', 'c', 'd'],
        'value': best_params,
        'description': [
            'Scaling factor',
            'Payload size exponent',
            'Send probability exponent',
            'Cluster size exponent'
        ]
    }).to_csv('model_parameters.csv', index=False)
    
    print("\nAnalysis complete. Results saved to files.")

