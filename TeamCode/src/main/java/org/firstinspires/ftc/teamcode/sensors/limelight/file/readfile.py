def runPipeline(image, llrobot):
    with open("1.txt", "r") as f:
        print(f.readline())
    return [], image, [0] * 8  # Final result with contours
