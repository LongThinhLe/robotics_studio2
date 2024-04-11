import math
import copy
import elkai

def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def reorder_curves_greedy(curves):
    ordered_curves = []
    remaining_curves = curves.copy()
    
    current_curve = remaining_curves.pop(0)  # Start with the first curve
    ordered_curves.append(current_curve)
    
    while remaining_curves:
        # Find the curve that minimizes the distance from the current curve's end
        best_fit = None
        best_distance = float('inf')
        reverse_best_fit = False

        for curve in remaining_curves:
            # Use attribute access for 'start' and 'end'
            direct_distance = distance(current_curve.end, curve.start)
            reversed_distance = distance(current_curve.end, curve.end)
            
            if direct_distance < best_distance:
                best_fit = curve
                best_distance = direct_distance
                reverse_best_fit = False
                
            if reversed_distance < best_distance:
                best_fit = curve
                best_distance = reversed_distance
                reverse_best_fit = True
        
        remaining_curves.remove(best_fit)
        
        if reverse_best_fit:
            # Reverse the curve's direction if necessary
            best_fit.reverse()
        
        ordered_curves.append(best_fit)
        current_curve = best_fit
    
    return ordered_curves

def create_distance_matrix(curves):
    n = len(curves) * 2  # Double the nodes for reversal options
    distance_matrix = [[0] * n for _ in range(n)]
    
    for i in range(len(curves)):
        for j in range(len(curves)):
            if i != j:  
                # Direct distances
                distance_matrix[2*i][2*j] = round(distance(curves[i].end, curves[j].start))
                distance_matrix[2*i+1][2*j+1] = round(distance(curves[i].start, curves[j].end))

                # Reversal distances
                distance_matrix[2*i][2*j+1] = round(distance(curves[i].end, curves[j].end))
                distance_matrix[2*i+1][2*j] = round(distance(curves[i].start, curves[j].start))
    
    return distance_matrix

def apply_tsp_solution_with_reversal(curves, tsp_solution):
    ordered_and_oriented_curves = []
    for node in tsp_solution:
        curve_index = node // 2
        curve = curves[curve_index]
        
        # Check if the curve should be reversed
        if node % 2 == 1:
            # If so, make a copy and reverse it 
            curve = copy.deepcopy(curve)  
            curve.reverse()
        
        ordered_and_oriented_curves.append(curve)
    
    return ordered_and_oriented_curves

def solve_tsp(curves):
    distance_matrix = create_distance_matrix(curves)
    tour = elkai.solve_int_matrix(distance_matrix)
    ordered_curves = apply_tsp_solution_with_reversal(curves, tour)
    
    return ordered_curves