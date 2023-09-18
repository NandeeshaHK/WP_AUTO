import cv2
import numpy as np

# Function to calculate the percentage of matched keypoints between two images
def calculate_matching_percentage(kp1, kp2, matches):
    matched_points = len(matches)
    total_points = max(len(kp1), len(kp2))
    matching_percentage = (matched_points / total_points) * 100
    print(matching_percentage)
    return matching_percentage

# Function to calculate the most dominant color in the input image
def calculate_dominant_color(image):
    # Calculate image dimensions and central region boundaries
    h, w, _ = image.shape
    central_range = int(w * 0.18)
    
    # Extract the central region of the image
    central_region = image[:, central_range : central_range * 2, :]
    
    # Compute the color histogram within the central range
    hist = cv2.calcHist([central_region], [0, 1, 2], None, [256, 256, 256], [0, 256, 0, 256, 0, 256])
    
    # Find the most frequent color
    max_freq = np.max(hist)
    dominant_color = np.unravel_index(np.argmax(hist), hist.shape)
    
    return dominant_color

# Function to generate modified images with dominant color
def generate_modified_images(input_image, dominant_color, diff_threshold=200):
    # Calculate RGB sum of dominant color
    dominant_sum = np.sum(dominant_color)
    
    # Calculate boundaries for nearest colors based on diff_threshold
    lower_bound = dominant_sum - diff_threshold
    upper_bound = dominant_sum + diff_threshold
    
    # Create masks for nearest colors
    lower_mask = np.sum(input_image, axis=2) >= lower_bound
    upper_mask = np.sum(input_image, axis=2) <= upper_bound
    
    # Retain nearest colors and convert the rest to black
    nearest_colors_image = input_image.copy()
    nearest_colors_image[lower_mask & upper_mask] = [0, 0, 0]
    nearest_colors_image[np.logical_not(lower_mask & upper_mask)] = input_image[np.logical_not(lower_mask & upper_mask)]

    
    # Resize the larger image to 1080p resolution
    scale_factor = 1080 / nearest_colors_image.shape[1]
    large_image2 = cv2.resize(nearest_colors_image, (1080, int(nearest_colors_image.shape[0] * scale_factor)))
            
    cv2.imshow('Nearest color image', large_image2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Keep only the nearest color to dominant color
    dominant_color_image = input_image.copy()
    dominant_color_image[lower_mask & upper_mask] = [0, 0, 0]
    dominant_color_image[np.logical_not(lower_mask & upper_mask)] = [255, 255, 255]
    
    #Resize the larger image to 1080p resolution
    scale_factor = 1080 / dominant_color_image.shape[1]
    large_image3 = cv2.resize(dominant_color_image, (1080, int(dominant_color_image.shape[0] * scale_factor)))
            
    cv2.imshow('Dominant color image', large_image3)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return nearest_colors_image, dominant_color_image

# Function to enhance object detection accuracy
def enhanced_object_detection(input_image_path, larger_image_path, min_matching_percentage=0):
    input_image = cv2.imread(input_image_path)
    larger_image = cv2.imread(larger_image_path)

    dominant_color = calculate_dominant_color(input_image)
    print(dominant_color)
    #for different color with letter in the image (R and R)
    # nearest_colors_image, dominant_color_image = generate_modified_images(input_image, dominant_color, 60)
    # nearest_colors_large_image, dominant_color_large_image = generate_modified_images(larger_image, dominant_color,5.5)

    nearest_colors_image, dominant_color_image = generate_modified_images(input_image, dominant_color, 65)
    nearest_colors_large_image, dominant_color_large_image = generate_modified_images(larger_image, dominant_color,150)

    modified_images = [nearest_colors_image, dominant_color_image]
    modified_large_images = [nearest_colors_large_image, dominant_color_large_image]
    orb = cv2.ORB_create()

    for idx, modified_image in enumerate(modified_images):
        for modified_large_image in modified_large_images:    
            gray_modified = cv2.cvtColor(modified_image, cv2.COLOR_BGR2GRAY)
            gray_larger = cv2.cvtColor(modified_large_image, cv2.COLOR_BGR2GRAY)

            kp1, des1 = orb.detectAndCompute(gray_modified, None)
            kp2, des2 = orb.detectAndCompute(gray_larger, None)

            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)

            matching_percentage = calculate_matching_percentage(kp1, kp2, matches)

            if matching_percentage >= min_matching_percentage:
                M, mask = cv2.findHomography(np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2),
                                            np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2),
                                            cv2.RANSAC, 5.0)

                h, w = gray_modified.shape
                corners = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                transformed_corners = cv2.perspectiveTransform(corners, M)
                avg_point = np.mean(transformed_corners, axis=0).reshape(-1, 2)
                input_resolution = (w, h)
                rectangle_size = int(max(input_resolution) * 0.5)

                x, y = avg_point[0]
                cv2.rectangle(larger_image, (int(x) - rectangle_size, int(y) - rectangle_size),
                            (int(x) + rectangle_size, int(y) + rectangle_size), (0, 0, 255), 2)

                cv2.circle(larger_image, (int(x), int(y)), 5, (0, 255, 0), -1)

                highlighted_image_path = f'intermediate_img/highlighted_rectangle_{idx}.jpg'
                highlighted_rectangle = larger_image[int(y) - rectangle_size:int(y) + rectangle_size,
                                    int(x) - rectangle_size:int(x) + rectangle_size]
                # cv2.imwrite(highlighted_image_path, highlighted_rectangle)

                x, y = avg_point[0]
                cv2.rectangle(larger_image, (int(x) - rectangle_size, int(y) - rectangle_size),
                            (int(x) + rectangle_size, int(y) + rectangle_size), (0, 0, 255), 2)

                cv2.circle(larger_image, (int(x), int(y)), 5, (0, 255, 0), -1)

                gray_larger = cv2.cvtColor(larger_image, cv2.COLOR_BGR2GRAY)
                # Resize the larger image to 1080p resolution
                scale_factor = 1080 / larger_image.shape[1]
                large_image2 = cv2.resize(larger_image, (1080, int(larger_image.shape[0] * scale_factor)))
                
                cv2.imshow(f'Detected Object {idx}', large_image2)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

# Test the enhanced algorithm with example images
input_image_path = '/home/nerdnhk/Image_Detection/cropped_img/Cropped_Y.jpg'
larger_image_path = '/home/nerdnhk/Image_Detection/Road Targets/164.jpg'
threshold = 0
enhanced_object_detection(input_image_path, larger_image_path)
