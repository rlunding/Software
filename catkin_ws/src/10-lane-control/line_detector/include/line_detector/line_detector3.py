import cv2
import numpy as np
import copy

from duckietown_utils.parameters import Configurable
from .line_detector_interface import (Detections, LineDetectorInterface)


class LineDetectorHLS(Configurable, LineDetectorInterface):
    """ LineDetectorHLS """

    def __init__(self, configuration):
        # Images to be processed
        self.bgr = np.empty(0)
        self.hls = np.empty(0)
        self.edges = np.empty(0)

        param_names = [
            'hls_white1',
            'hls_white2',
            'hls_yellow1',
            'hls_yellow2',
            'hls_red1',
            'hls_red2',
            'dilation_kernel_size',
            'canny_thresholds',
            'hough_threshold',
            'hough_min_line_length',
            'hough_max_line_gap',
        ]
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self, param_names, configuration)

    def _colorFilter(self, color):
        # threshold colors in HSV space
        if color == 'white':
            bw = cv2.inRange(self.hls, self.hls_white1, self.hls_white2)
        elif color == 'yellow':
            bw = cv2.inRange(self.hls, self.hls_yellow1, self.hls_yellow2)
        elif color == 'red':
            bw = cv2.inRange(self.hls, self.hls_red1, self.hls_red2)
        else:
            raise Exception('Error: Undefined color strings...')

        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)
        
        # refine edge for certain color
        edge_color = cv2.bitwise_and(bw, self.edges)

        return bw, edge_color

    def _findEdge(self, gray):
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize=3)
        return edges

    def _HoughLine(self, edge):
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, self.hough_threshold, np.empty(1), self.hough_min_line_length, self.hough_max_line_gap)
        if lines is not None:
            lines = np.array(lines[:, 0])
        else:
            lines = []
        return lines
    
    def _checkBounds(self, val, bound):
        val[val < 0] = 0
        val[val >= bound] = bound-1
        return val

    def _correctPixelOrdering(self, lines, normals):
        flag = ((lines[:, 2]-lines[:, 0])*normals[:, 1] - (lines[:, 3]-lines[:, 1])*normals[:, 0]) > 0
        for i in range(len(lines)):
            if flag[i]:
                x1, y1, x2, y2 = lines[i, :]
                lines[i, :] = [x2, y2, x1, y1]
 
    def _findNormal(self, bw, lines):
        normals = []
        centers = []
        if len(lines) > 0:
            length = np.sum((lines[:, 0:2] - lines[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1. * (lines[:, 3:4]-lines[:, 1:2])/length
            dy = 1. * (lines[:, 0:1]-lines[:, 2:3])/length

            centers = np.hstack([(lines[:, 0:1]+lines[:, 2:3])/2, (lines[:, 1:2]+lines[:, 3:4])/2])
            x3 = (centers[:, 0:1] - 3.*dx).astype('int')
            y3 = (centers[:, 1:2] - 3.*dy).astype('int')
            x4 = (centers[:, 0:1] + 3.*dx).astype('int')
            y4 = (centers[:, 1:2] + 3.*dy).astype('int')
            x3 = self._checkBounds(x3, bw.shape[1])
            y3 = self._checkBounds(y3, bw.shape[0])
            x4 = self._checkBounds(x4, bw.shape[1])
            y4 = self._checkBounds(y4, bw.shape[0])
            flag_signs = (np.logical_and(bw[y3, x3] > 0, bw[y4, x4] == 0)).astype('int')*2-1
            normals = np.hstack([dx, dy]) * flag_signs

            self._correctPixelOrdering(lines, normals)
        return centers, normals

    def detectLines(self, color):
        bw, edge_color = self._colorFilter(color)
        lines = self._HoughLine(edge_color)
        centers, normals = self._findNormal(bw, lines)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)

    def setImage(self, bgr):
        self.bgr = np.copy(bgr)
        self.hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        self.edges = self._findEdge(self.bgr)
  
    def getImage(self):
        return self.bgr
