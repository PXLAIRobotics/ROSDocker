
import numpy as np
import cv2

class Example1:
    def __init__(self, image_name):
        self.image_name = image_name

        # Orange  0-22
        # # Yellow 22- 38
        # # Green 38-75
        # Blue 75-130
        # Violet 130-160
        # Red 160-179
        self.bound_low1 = np.array([0, 0, 0])
        self.bound_up1  = np.array([0, 0, 0])

        self.bound_low2 = np.array([0, 0, 0])
        self.bound_up2  = np.array([0, 0, 0])

    def load_and_show_image(self):
        # Load an color image
        #
        # - cv2.IMREAD_COLOR: Loads a color image. Any transparency of image 
        #                     will be neglected. It is the default flag.
        # - cv2.IMREAD_GRAYSCALE: Loads image in grayscale mode.
        # - cv2.IMREAD_UNCHANGED: Loads image as such including alpha channel.
        self.image = cv2.imread(self.image_name, cv2.IMREAD_UNCHANGED)
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def conver_to_hsv(self):
        self.image_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        cv2.namedWindow("hsv_image", cv2.WINDOW_NORMAL)
        cv2.imshow("hsv_image", self.image_hsv)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def set_mask(self):
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", self.image)

        cv2.namedWindow("hsv_image", cv2.WINDOW_NORMAL)
        cv2.imshow("hsv_image", self.image_hsv)

        cv2.namedWindow("mask1")
        cv2.createTrackbar("Hue lower bound:", "mask1", 0, 179, self.callback_trackbars)
        cv2.createTrackbar("Hue upper bound:", "mask1", 0, 179, self.callback_trackbars)
        cv2.createTrackbar("Saturation lower bound: ", "mask1", 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Saturation upper bound: ", "mask1", 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Value lower bound:", "mask1", 0, 255, self.callback_trackbars)
        cv2.createTrackbar("Value upper bound:", "mask1", 0, 255, self.callback_trackbars)

        while True:
            mask1 = cv2.inRange(self.image_hsv, self.bound_low1, self.bound_up1)
            cv2.imshow('mask1', mask1)

            key = cv2.waitKey(33)
            if key == 27:    # Esc key to stop
                break

        cv2.destroyAllWindows()

    def callback_trackbars(self, value):
        h_low1 = cv2.getTrackbarPos('Hue lower bound:', 'mask1')
        h_up1  = cv2.getTrackbarPos('Hue upper bound:', 'mask1')
        s_low1 = cv2.getTrackbarPos('Saturation lower bound:', 'mask1')
        s_up1  = cv2.getTrackbarPos('Saturation upper bound:', 'mask1')
        v_low1 = cv2.getTrackbarPos('Value lower bound:', 'mask1')
        v_up1  = cv2.getTrackbarPos('Value upper bound:', 'mask1')

        self.bound_low1 = np.array([h_low1, s_low1, v_low1], np.uint8)
        self.bound_up1  = np.array([h_up1, s_up1, v_up1], np.uint8)


if __name__ == "__main__":
    example1 = Example1("../../Images/strawberries-red-fruit-organic.jpg")
    example1.load_and_show_image()
    example1.conver_to_hsv()
    example1.set_mask()
