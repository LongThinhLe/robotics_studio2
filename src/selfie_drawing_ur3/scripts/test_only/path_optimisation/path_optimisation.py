
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

import math

def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)