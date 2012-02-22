import sys

def rgb2hue(r,g,b):
    """Compute hue / 2 (to be used as an OpenCV threshold)."""
    h = 60.0 * (((b - r) / (g - min(r,b)))+2.0)
    return (h/2)

if __name__=="__main__":
    print sys.argv
    if len(sys.argv) != 4:
        print("Usage: rgb2hue.py red green blue")
    else:
        print(rgb2hue(*[float(x) for x in sys.argv[1:]]))
