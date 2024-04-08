
def reorder_curves_greedy(curves):
    ordered_curves = []
    remaining_curves = curves.copy()
    
    current_curve = remaining_curves.pop(0)  # Start with the first curve
    ordered_curves.append(current_curve)
    
    while remaining_curves:
        closest_curve = min(remaining_curves, key=lambda curve: distance(current_curve.end, curve.start))
        remaining_curves.remove(closest_curve)
        ordered_curves.append(closest_curve)
        current_curve = closest_curve
    
    return ordered_curves

import math

def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)