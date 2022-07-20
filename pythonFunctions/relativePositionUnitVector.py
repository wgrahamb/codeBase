from unitvector import unitvector

# Input: must be numpy arrays, equivalent shapes, and of the float type. (position vector)
# Output: yields line of sight from a1 to a2.
def losufd(a1, a2):
    rel = a2 - a1
    return unitvector(rel)
