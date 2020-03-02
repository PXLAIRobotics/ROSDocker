import  os
from std_msgs.msg import String
from sensor_msgs.msg import Image

import sys
from PyQt4 import QtGui
from PyQt4 import QtCore

import rospy

CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms




class Example(QtGui.QWidget):

    def __init__(self):
        super(Example, self).__init__()

        subscriberName = "test_subscriber_" + str(os.getpid())
        topicName = "test_topic"

        rospy.init_node(subscriberName, anonymous=False)

        subscriber = rospy.Subscriber(topicName, String, self.callbackFunction)
        self.publisher = rospy.Publisher(topicName, String, queue_size=1)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.redraw_callback)
        timer.start(GUI_UPDATE_PERIOD)

        self.initUI()
        
    def redraw_callback(self):
        print "Redraw"

    def initUI(self):
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))

        self.setToolTip('This is a <b>QWidget</b> widget')

        btn = QtGui.QPushButton('Button', self)
        btn.setToolTip('This is a <b>QPushButton</b> widget')
        btn.clicked.connect(self.clicked)
        btn.resize(btn.sizeHint())
        btn.move(50, 50)

        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Tooltips')
        self.show()

    def clicked(self):
        publisherName = "test_publisher_" + str(os.getpid())
        topicName = "test_topic"
        data = publisherName + ":data:"
        rospy.loginfo("Sending: " + data)
        self.publisher.publish(data)
        rospy.loginfo("Sent\n")


    def callbackFunction(self, data):
        rospy.loginfo(data.data)


def main():
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()