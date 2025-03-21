with open("dropzone_log.txt", "a") as f:
    f.write(f"{time.ctime()}: DropZone detected at Lat: {lat}, Lon: {lon}, Alt: {rel_alt}\n")