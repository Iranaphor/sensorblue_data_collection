import yaml
import rospy

SENSORS = {
    'r0.7-ca':[],
    'r0.7-cb':['s1', 's2'],
    'r0.7-c0':['s3'],
    'r0.7-c1':['s4']
}

class SensorBlueActionServer():
    def __init__():
        # Get location information
        self.closest_node = None
        self.clo_sub = self.subscriber('/closest_node', self.clo_cb)
        self.current_node = None
        self.cur_sub = self.subscriber('/current_node', self.cur_cb)

        # Require the topological map
        self.tmap = None
        self.map_sub = self.subscriber('/topological_map_2', self.map_cb)

        # Configure connections for action server
        self.goal_sub = self.subscriber('/sensorblue/goal', self.goal_cb)
        self.cancel_sub = self.subscriber('/sensorblue/cancel', self.cancel_cb)
        self.status_pub = self.publisher('/sensorblue/status')
        self.feedback_pub = self.publisher('/sensorblue/feedback')
        self.result_pub = self.publisher('/sensorblue/result')

    # Use these to compare our location with what sensors to collect from
    def clo_cb(self, msg): self.closest_node = msg.data
    def cur_cb(self, msg): self.current_node = msg.data

    # Get map to use for routing down row
    def map_cb(self, msg):
        self.map = yaml.safe_load(msg.data)
        self.search = TopologicalRouteSearch(self.map)

    # Use this to cancel the active goal (it will never be called)
    def cancel_cb(self, msg): pass

    def goal_cb(self, msg):
        row_name = msg.data
        first_node = row_name+"-cb"
        last_node = row_name+"-cy"

        # Assume when this has been recieved we are are r0.7-ca
        if not self.closest_node.startswith(row_name):
            print('Not at the right node!')
            return

        # Started off being on the entry node
        # loop through each node along the row, till getting to the final node
        # each time we set a new goal on the route

        # 1. find route along row using toponav utils
        route = self.search.search_route(first_node, last_node)

        # 2. begin a for loop
        for node in self.search.nodes:
            #a. navigate to ith node on the route provided by toponav
            self.perform_navigation(node)
            self.status_pub.publish(Status(data="new node arrived"))

            #b. on nav==complete: trigger node-data-collection
            data = self.perform_data_collection(node)
            self.status_pub.publish(Status(data="new data added"))

            #d. repeat


    # PUBLISH NAVIGATION GOAL, THEN WAIT TILL A STATUS CALLBACK
    def perform_navigation(self, node):
        self.toponav_status = None
        self.toponav_goal_pub.publish(data=node)
        while not self.toponav_status == True:
            sleep(0.1)
    def toponav_status_cb(self, msg):
        self.toponav_status = True


    # BEGIN SCANNING SENSORS
    def perform_data_collection(self, node):
        for sensor in SENSORS[node]:
            print('scanning...')
