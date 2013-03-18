#!/usr/bin/env python
import roslib; roslib.load_manifest("kitchencommunication")
import rospy
import time
import tf
import math
import subprocess
import os

from std_msgs.msg import String, Empty, Int16, Header
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from kitchencommunication.srv import *
from json_prolog.srv import *
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID


parser_path = ""


# Publisher to make the robot speak
pub = rospy.Publisher("robot_response", String)
pub_log = rospy.Publisher("gui_log", String)
pub_ps = rospy.Publisher("move_base_simple/goal", PoseStamped)
pub_goal = rospy.Publisher("move_base/goal", MoveBaseActionGoal)
pub_cancel = rospy.Publisher("move_base/cancel", GoalID)
pubTalk = rospy.Publisher("activate_speech", Int16)
pub_startstop = rospy.Publisher("startstop", Int16)
# Positions of the robot and the human (x, y, z, rot-angle)
mypos = [0.0, 0.0, 0.0, 0.0]
humanpos = [0.0, 0.0, 0.0, 0.0]


robot_stopped = False

# goal id to stop the movement
last_goal_id = None


# Send a query to json_prolog and returns a list of results
# req: String
def send_query(req):
    rospy.wait_for_service('json_prolog/simple_query')
    try:
        handle = rospy.ServiceProxy('json_prolog/simple_query', PrologQuery)
        prolog_id = "sr%f" % time.time()
        qry = handle(0, prolog_id, req)
        res_list = []
        if (qry.ok == True):
            sol = rospy.ServiceProxy('json_prolog/next_solution', PrologNextSolution)
            res = sol(prolog_id)
            while (res.status != 0):
                if (res.status == 3):
                    res_list.append(res.solution)
                else:
                    res_list.append("Failure")
                res = sol(prolog_id)
            return res_list
        else:
            res_list.append("Failure")
            print "Query '"+req+"' not okay"
            return res_list
    except rospy.ServiceException, e:
        print "Prolog query failed: %s"%e


def startstop(msg):
    if (msg.data == 0):
        stop_all()
    else:
        global robot_stopped
        robot_stopped = False
        print "Robot started..."
    return


# Returns the position of the robot (x, y, z, rotation)
def setmypos(p):
    global mypos
    mypos = [p.data.pose.pose.position.x, p.data.pose.pose.position.y,
        p.data.pose.pose.position.z,
        tf.transformations.euler_from_quaternion(p.data.pose.pose.orientation)[2]]


def my_pos():
    # amcl_pose
    # /Human/Pose nav_msgs/odometry, geometry_msgs/PoseWithCovarianceStamp
    global mypos
    return mypos


def sethumanpos(p):
    global humanpos
    humanpos = [p.data.pose.pose.position.x, p.data.pose.pose.position.y,
        p.data.pose.pose.position.z,
        tf.transformations.euler_from_quaternion(p.data.pose.pose.orientation)[2]]


def human_pos():
    global humanpos
    return humanpos


# Stops the robot
def stop_all():
    if(not last_goal_id == None):
        pub_cancel.publish(last_goal_id)
    global robot_stopped
    robot_stopped = True
    pub_log.publish(rospy.get_name()+": Movement stopped")
    pub.publish(String("Everything stopped!"))


# Determines the coordinates of the object in the semantic map
# obj: string
# return: five floats (position of obj + cos/sin of rotation angle)
def find_coordinates(obj):
    res = send_query("owl_has('"+obj+"', 'http://ias.cs.tum.edu/kb/knowrob.owl#hingedTo', O)")
    if(len(res)>0):
        dummy = res[0]
        dummy = dummy[6:len(dummy)-2]
    else:
        dummy = obj
    res2 = send_query("owl_has(S, knowrob:objectActedOn, '"+dummy+"')")
    SemanticMapPerception = res2[0]
    res3 = send_query("owl_has('"+SemanticMapPerception[6:len(SemanticMapPerception)-2]+"', knowrob:eventOccursAt, O)")
    matrix = res3[0]
    # Ask for matrix entries
    res4 = send_query("owl_has('"+matrix[6:len(matrix)-2]+"', P, O)")
    posx = 0.0
    posy = 0.0
    posz = 0.0
    cos_angle = 0.0
    sin_angle = 0.0
    for i in range(16):
        dummy = res4[i].split('"')
        if(dummy[3].endswith("m03")):
            posx = float(dummy[len(dummy)-2])
        elif(dummy[3].endswith("m13")):
            posy = float(dummy[len(dummy)-2])
        elif(dummy[3].endswith("m23")):
            posz = float(dummy[len(dummy)-2])
        elif(dummy[3].endswith("m00")):
            cos_angle = float(dummy[len(dummy)-2])
        elif(dummy[3].endswith("m01")):
            sin_angle = float(dummy[len(dummy)-2])
    return [posx, posy, posz, cos_angle, sin_angle]



# Sorts objects by the distance to the robot
def sort_by_distance(lst):
    n = len(lst)
    if (n > 10):
        n = 10
    lst = lst[0:n]
    erg = []
    for i in range(n):
        xyz = find_coordinates(lst[i])
        dist = math.sqrt( (xyz[0]-my_pos()[0])*(xyz[0]-my_pos()[0]) +
            (xyz[1]-my_pos()[0])*(xyz[1]-my_pos()[1]) )
        erg.append( (lst[i], dist) )
    erg = sorted(erg, key=lambda dst: dst[1])
    for i in range(n):
        lst[i] = erg[i][0]
    return lst



# Ask the user to specify an exact object
# list: list of objects to decide
def decide(list):
    answ = "I found more than one solution - which number did you mean?"
    i = 0
    list = sort_by_distance(list)
    while(i<len(list)):
        answ = answ+("\n  %i: "% i)+list[i]
        # visualize object
        #send_query("add_object("+list[i]+", $C)")
        i=i+1
    pub_log.publish(rospy.get_name()+": %i objects found in semantic map"%len(list))
    pub.publish(String(answ))
    rec = speech_recognition().lower()
    if(rec.startswith("Number ")):
        rec = erg.answer[7:len(rec)]
    print "Rec: "+rec
    if(rec == "zero"):
        return list[0]
    elif(rec == "one"):
        return list[1]
    elif(rec == "two"):
        return list[2]
    elif(rec == "three"):
        return list[3]
    elif(rec == "four"):
        return list[4]
    elif(rec == "five"):
        return list[5]
    elif(rec == "six"):
        return list[6]
    elif(rec == "seven"):
        return list[7]
    elif(rec == "eight"):
        return list[8]
    elif(rec == "nine"):
        return list[9]
    else:
        pub_log.publish(rospy.get_name()+": No number between 0 and 9")
        pub.publish(String("Sorry, I couldn't understand you."))
        return ""


# Finds an object in the semantic map
# obj: string
def find_object(obj):
    res_list = send_query("owl_has(A, rdf:type, knowrob:'"+obj.capitalize()+"')")#, add_object_with_children(A, $C)")
    # Search also for words starting with a capital letter
    obj = obj.lower()
    res_list.extend(send_query("owl_has(A, rdf:type, knowrob:'"+obj+"')"))
    i = 0
    while(i<len(res_list)):
        dummy=res_list[i]
        dummy=dummy[6:len(dummy)-2]
        res_list[i] = dummy
        i=i+1
    print "Number of results: %u"%len(res_list)
    print res_list
    if(len(res_list)==0):
        return ""
    elif(len(res_list)==1):
        return res_list[0]
    else:
        return decide(res_list)


# Sends a goal to move_base
# posx, posy, rotationz: float
def send_poseStamped(posx, posy, rotationz):
    return send_move_base_goal(posx, posy, rotationz)

    #global header_id
    #header_id = header_id+1
    #stamp = rospy.Time.now()
    #h = Header(header_id, stamp, "map")
    #p = Point(posx, posy, 0.0)
    #q = tf.transformations.quaternion_from_euler(0, 0, rotationz)
    #pose = Pose(p, q)
    ps = PoseStamped()#h, pose)
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position.x = posx
    ps.pose.position.y = posy
    dummy = tf.transformations.quaternion_from_euler(0, 0, rotationz)
    ps.pose.orientation.x = dummy[0]
    ps.pose.orientation.y = dummy[1]
    ps.pose.orientation.z = dummy[2]
    ps.pose.orientation.w = dummy[3]
    print ps
    pub_ps.publish(ps)
    print ("published goal pose")


def send_move_base_goal(posx, posy, rotationz):
    global last_goal_id
    ag = MoveBaseActionGoal()
    ag.header = Header()
    ag.goal_id = GoalID()
    ID = "0"
    if(not last_goal_id == None):
        print "Last goal id:"
        print last_goal_id
        IDint = int(float(last_goal_id.id))+1
        ID = str(IDint)
        print "ID:"
        print ID
        print "\n"
        #ag.header.seq = ID
    ag.goal_id.id = ID
    print ag.goal_id
    ag.goal = MoveBaseGoal()
    ag.goal.target_pose.header.frame_id = "map"
    ag.goal.target_pose.pose.position.x = posx
    ag.goal.target_pose.pose.position.y = posy
    dummy = tf.transformations.quaternion_from_euler(0, 0, rotationz)
    ag.goal.target_pose.pose.orientation.x = dummy[0]
    ag.goal.target_pose.pose.orientation.y = dummy[1]
    ag.goal.target_pose.pose.orientation.z = dummy[2]
    ag.goal.target_pose.pose.orientation.w = dummy[3]
    print ag
    pub_goal.publish(ag)
    last_goal_id = ag.goal_id
    pub_log.publish(rospy.get_name()+": published ActionGoal")


# Calls other procedure to fetch obj
# obj: Identifier of the object
def fetch(obj):
    print "Call other procedure to reach the searched object"


# Parses the command via Stanford Parser
# cmd: String
def parse_command(cmd):
    # Write command to dummy file
    parse_file = open(".parse.txt", "w")
    parse_file.write(cmd)
    parse_file.close()
    # Start the parser
    pub_log.publish(rospy.get_name()+": command received, starting parser...")
    global parser_path
    p = subprocess.Popen([parser_path+"/lexparser.sh", "./.parse.txt"], stdout=subprocess.PIPE)
    txt, err = p.communicate()
    lines = txt.split("\n")
    erg = []
    if(len(lines) < 2):
        print "Command couldn't be parsed."
        return []
    l = lines[1].strip()
    if(l == "(SBARQ"):
        qword = lines[2].strip().split()[2]
        qword = qword[0:len(qword)-2].lower()
        erg.append("question:"+qword)
    elif((l == "(S" or l == "(FRAG") and lines[2].strip().startswith("(VP")):
        erg.append("command")
    elif(l == "(NP"):
        erg.append("command")
    else:
        print "Neither command nor question! -"+lines[1].strip()+"-"
        return []
    # Get to the "interesting lines"
    i = 0
    while(i < len(lines) and not lines[i].startswith("root")):
        i = i+1
    root_i = i
    root = lines[i].split()[1].lower()
    root = root[0:len(root)-3]
    if (root.startswith("go") or root.startswith("move") or root.startswith("come") or
        root.startswith("get") or root.startswith("walk")):
        erg.append("go")
    elif (root.startswith("follow")):
        erg.append("follow")
    elif(root.startswith("stop") or root.startswith("halt") or root.startswith("don't move")
        or root.startswith("wait")):
        erg.append("stop")
    elif(root.startswith("fetch") or root.startswith("bring") or root.startswith("give") or
        root.startswith('pick') or root.startswith("take")):
        erg.append("fetch")
    elif(root.startswith("set") or root.startswith("lay")):
        erg.append("set")
    elif(root.startswith("open")):
        erg.append("open")
    elif(root.startswith("close")):
        erg.append("close")
    elif(root.startswith("cook") or root.startswith("make") or root.startswith("prepare")):
        erg.append("cook")
    elif(root.startswith("find") or root.startswith("search")):
        erg.append("find")
    elif(root.startswith("empty") or root.startswith("unload")):
        erg.append("empty")
    elif(root.startswith("turn on") or root.startswith("switch on")):
        erg.append("turnon")
    elif(root.startswith("turn off") or root.startswith("switch off")):
        erg.append("turnoff")
    elif(root.startswith("clean") or root.startswith("wash")):
        erg.append("clean")
    elif(root.startswith("is") or root.startswith("are") or root.startswith("'s")):
        erg.append("is")
    else:
        print "No known command."
        return []
    i = i+1
    # Add further objects etc.
    while(i < len(lines) and not lines[i] == ""):
        dummy = lines[i].split("(")
        typ = dummy[0]
        dummy = dummy[1].split("-")
        value = dummy[1][3:len(dummy[1])]#dummy[0]+"-"+dummy[1][3:len(dummy[1])]
        if(typ == "conj_and" and lines[root_i].split(",")[1][1:-3] == dummy[0]):
            erg.append("root:"+dummy[1][3:len(dummy[1])])
        elif(erg[0] == "question:where" and typ == "nsubj" or
                typ == "dobj" or typ == "prep_to" or typ == "dep"):
            erg.append(typ+":"+value)
        i = i+1
    print "Command:"
    print erg
    # remove dummy file
    os.remove(".parse.txt")
    return erg


# Sends a poseStamped to move the robot to the object
# obj: identifier of existing object in the semantic map
def move_to_object(obj):
        [posx, posy, posz, cos_angle, sin_angle] = find_coordinates(obj)
        # Compute height of object (= distance to stop in front of the object)
        # TODO: Unterscheidung zw. Tuer und Drawer etc.
        qrylen = send_query("owl_has('"+obj+"', knowrob:depthOfObject, O)")
        dummy = qrylen[0].split('"')
        depth = float(dummy[len(dummy)-2])
        qrywidth = send_query("owl_has('"+obj+"', knowrob:widthOfObject, O)")
        dummy = qrywidth[0].split('"')
        width = float(dummy[len(dummy)-2])
        # Add half the depth to the mid-point
        posx = posx - cos_angle*(depth/2)
        posy = posy - sin_angle*(depth/2)
        # Keep some distance between the robot and the object to open doors etc.
        posx = posx+cos_angle#width
        posy = posy+sin_angle#width
        send_poseStamped(posx, posy, math.acos(cos_angle)+math.pi)



# Analyzes the user command to determine the action
# query: string
def determine_action(query):
    if (query.lower().startswith("stop") or query.lower().startswith("halt")):
        pub_startstop.publish(0)
        stop_all()
        determine_action(speech_recognition())
    # Parse the command
    determine_action_helper(parse_command(query))
    determine_action(speech_recognition())



# Receives a list of parsed words and determines the goal
# parsed_query: list[String]
def determine_action_helper(parsed_query):
    # Argument index
    i = 2
    if(len(parsed_query) == 0):
        print "Parser couldn't determine action"
        pub.publish("I'm sorry, but I can't understand what you are saying.")
    elif(parsed_query[0] == "question:where"):
        args = parsed_query[2].split(":")
        obj = ""
        if(args[0] == "nsubj"):
            obj = args[1].split("-")[0]
        ret = find_object(obj)
        if(ret is ""):
            pub.publish(String("I couldn't find the "+obj+" in the kitchen."))
        else:
            [posx, posy, posz, cos, sin] = find_coordinates(ret)
            print "The "+obj+" is here: %f/%f"%(posx, posy)
            pub.publish(String("The "+obj+" is here: %f/%f"%(posx, posy)))
    elif(parsed_query[0] is "command"):
        if(parsed_query[1] == "go"):
            arg = parsed_query[2].split(":")
            obj = ""
            ret = ""
            while (arg[0] != "prep_to" and arg[0] != "root" and len(parsed_query)>i+1):
                i = i+1
                arg = parsed_query[i].split(":")
#            length = 1
#            if(arg[1] is "left"):
#                send_poseStamped(my_pos()[0]+math.cos(my_pos()[3]+math.PI/2)*length,
#                    my_pos()[1]+math.sin(my_pos()[3]+math.PI/2)*length, 0, my_pos[3]+math.PI/2)
#            elif(arg[1] is "right"):
#                send_poseStamped(my_pos()[0]+math.cos(my_pos()[3]-math.PI/2)*length,
#                    my_pos()[1]+math.sin(my_pos()[3]-math.PI/2)*length, 0, my_pos[3]-math.PI/2)
#            elif(arg[1] is "forward" or arg[1] is "ahead" or arg[1] is "along"):
#                send_poseStamped(my_pos()[0]+math.cos(my_pos()[3])*length,
#                    my_pos()[1]+math.sin(my_pos()[3])*length, 0, my_pos[3])
#            else:
            obj = arg[1]#.split("-")[0]
            ret = find_object(obj)
            if(ret is ""):
                pub.publish(String("I couldn't find the "+obj+"."))
            else:
                pub.publish(String("I'll go to the "+obj))
                move_to_object(ret)
        elif(parsed_query[1] == "follow"):
            if(parsed_query[2].split(":")[0].startswith("dobj") and
                    parsed_query[2].split(":")[1] != "me"):
                pub.publish(String("I'll follow "+parsed_query[2].split(":")[1]))
            else:
                pub.publish(String("I'll follow you."))
                for i in range(10):
                    send_poseStamped(human_pos()[0], human_pos()[1], human_pos()[3])
                    time.sleep(1)
        elif(parsed_query[1] == "stop"):
            stop_all()
        elif(parsed_query[1] == "fetch"):
            # TODO: evtl. to-Parameter beachten
            arg = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dobj" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = arg[1].split("-")[0]
            ret = find_object(obj)
            if(ret is ""):
                pub.publish(String("I don't know where the "+obj+" is. Where shall I go to fetch it?"))
            else:
                pub.publish(String("I'll fetch the "+obj))
                move_to_object(ret)
                fetch(ret)
        elif(parsed_query[1] == "set"):
            if(parsed_query[2].startswith("dobj:table")):
                pub.publish("How many plates do you need?")
                number = speech_recognition()
                pub.publish("I'll set the table for "+number+" persons")
            else:
                pub.publish("Sorry, I can't understand your command.")
        elif(parsed_query[1] == "open" or parsed_query[1] == "close"):
            arg = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dobj" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = arg[1].split("-")[0]
            ret = find_object(obj)
            if(ret is ""):
                pub.publish(String("Where is the "+obj+"? I couldn't find it!"))
            else:
                # TODO: Test ob Angel vorhanden
                pub.publish(String("I'll go to the "+obj+" and "+parsed_query[1]+" it"))
                move_to_object(ret)
        elif(parsed_query[1] == "cook"):
            pub.publish("Sorry, I don't know how to cook. You'll have to say step by step what I have to do.")
        elif(parsed_query[1] == "find"):
            args = parsed_query[2].split(":")
            obj = ""
            if(args[0] == "dobj"):
                obj = args[1].split("-")[0]
            ret = find_object(obj)
            if(ret is ""):
                pub.publish(String("I couldn't find the "+obj+"."))
            else:
                [posx, posy, posz, cos, sin] = find_coordinates(ret)
                print "The "+obj+" is here: %f/%f"%(posx, posy)
                pub.publish(String("The "+obj+" is here: %f/%f"%(posx, posy)))
        elif(parsed_query[1] == "empty"):
            args = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dobj" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = args[1].split("-")[0]
            ret = find_object(obj)
            if(ret is ""):
                pub.publish(String("I couldn't find the "+obj+"."))
            else:
                pub.publish(String("I'll empty the "+obj+"."))
                move_to_object(ret)
        elif(parsed_query[1] == "turnon"):
            args = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dobj" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = args[1].split("-")[0]
            pub.publish("I'll turn the "+obj+" on!")
        elif(parsed_query[1] == "turnoff"):
            args = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dobj" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = args[1].split("-")[0]
            pub.publish("I'll turn the "+obj+" off!")
        elif(parsed_query[1] == "wash"):
            args = parsed_query[2].split(":")
            i = 2
            while(arg[0] != "dep" and arg[0] != "root"):
                i = i+1
                arg = parsed_query[i].split(":")
                if(len(parsed_query) >= i):
                    break
            obj = args[1].split("-")[0]
            pub.publish("I'll wash the "+obj+"!")
        else:
            pub.publish("Sorry, but I can't understand your command!")
    else:
        pub.publish("I received: '"+query+"' but I don't understand it.")
    while(i+1 < len(parsed_query) and not parsed_query[i].startswith("root")):
        i = i+1
    if(i < len(parsed_query) and parsed_query[i].startswith("root")):
        r = ["command", parsed_query[i].split(":")[1]]
        print parsed_query[i+1:len(parsed_query)]
        r.extend(parsed_query[i+1:len(parsed_query)])
        print r
        determine_action_helper(r)


# Receive an user command from the stt- or gui-node
def speech_recognition():
    # receive speech commands
    global pubTalk
    pubTalk.publish(1)
    received = False
    erg = sttResponse("")
    global robot_stopped
    while(robot_stopped):
        time.sleep(0.1)
    while(not received):
        try:
            srv_stt = rospy.ServiceProxy('/record_speech', stt)
            erg = srv_stt()
        except rospy.ServiceException, e:
            print "Speech-To-Text service unavailable. Use text input..."
            try:
                rospy.wait_for_service('/record_text')
                srv_stt = rospy.ServiceProxy('/record_text', stt)
                erg = srv_stt()
            except rospy.ServiceException, e:
                print "Exception raised: %s"%e
        if (erg.answer is "We could not recognize the sentence!"):
            pub.publish(String("I didn't understand you, could you say it again, please?"))
            print "Sentence not understood"
        else:
            print "Received: "+erg.answer
            received = True
    pubTalk.publish(0)
    return erg.answer


if __name__ == "__main__":
    # Warning, if parser directory is not set
    if(parser_path == ""):
        print "Parser path missing."
    rospy.init_node('ki')
    s = rospy.Service('ki/start_stop', start_stop, startstop)
    rospy.Subscriber("amcl_pose", Odometry, setmypos)
    rospy.Subscriber("Human/Pose", PoseWithCovarianceStamped, sethumanpos)
    send_query("register_ros_package(ias_semantic_map)")
    print "registered ros-package ias_semantic_map at json_prolog"
    send_query("visualisation_canvas(C)")
    print "visualisation initialized"
    determine_action(speech_recognition())

