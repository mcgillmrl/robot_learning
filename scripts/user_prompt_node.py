#!/usr/bin/env python
import rospy
import rospkg
import os
from std_msgs.msg import String, Empty
from sensor_msgs.msg import JoyFeedbackArray, JoyFeedback
import threading
import pygame

class UserPrompt():
  # place compound names before single names
  valid_task_types = ['bank', 'changedepth', 'circlecorkscrew', \
    'corkscrewfast', 'corkscrewslow', 'crooked', 'planarcircle', \
    'pray', 'uturnbellyup', 'uturnknifeedge', 'bellyup', 'knifeedge']
  # NOTE: sounds obtained from http://soundoftext.com
  
  def __init__(self):
    rp = rospkg.RosPack()
    
    self.marshall_mode = ''
    self.accept_prompt = False
    
    self.rumble_type = None # possible values: long, shortx3, short
    self.rumble_thread = None
    
    self.sound_enabled = False
    self.sound_thread = None
    self.sound_lock = threading.Lock()
    self.music_dir = os.path.join(rp.get_path('robot_learning'), 'mp3')
    pygame.mixer.init()
    
    rospy.init_node('user_prompt_listener', anonymous=True, disable_signals=True)
    self.restart_pub = rospy.Publisher('/rl/trigger_start', Empty, queue_size=10)
    self.rumble_pub = rospy.Publisher('/rumble_event_node/set_rumble', JoyFeedbackArray, queue_size=10)
    self.marshall_mode_sub = rospy.Subscriber('/rl_marshall/mode', String, self.callback_marshall_mode)
    self.user_prompt_sub = rospy.Subscriber('/rl/user_prompt', String, self.callback_prompt)
    
    rospy.loginfo('user prompt node initialized')

  def play_sounds(self, files):
    self.sound_lock.acquire()
    
    first = True
    for f in files:
      if not self.sound_enabled:
        break
      
      # Sleep a bit between playback
      if first:
        first = False
      #else:
      #  pygame.time.delay(500)

      if not self.sound_enabled:
        break
      
      # Load and play sound
      filepath = os.path.join(self.music_dir, f)
      if os.path.exists(filepath):
        pygame.mixer.music.load(filepath)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
          pygame.time.delay(100)
      else:
        rospy.logwarn('Could not load ' + filepath)

    self.sound_lock.release()

  def restart_sound_thread(self, task_type):
    if self.sound_thread is not None:
      self.sound_enabled = False
      self.sound_thread.join()
      self.sound_thread = None
    self.sound_enabled = True
    self.sound_thread = threading.Thread(target=self.play_sounds, kwargs={'files': ['next_task.mp3', task_type+'.mp3', 'next_task.mp3', task_type+'.mp3']})
    self.sound_thread.start()
    
  def announce_task(self, prompt):
    start_idx = prompt.find('[')
    if start_idx < 0:
      rospy.logwarn('Failed to parse task (1): ' + prompt)
      return
    end_idx = prompt.find(']', start_idx)
    if end_idx < 0:
      rospy.logwarn('Failed to parse task (2): ' + prompt)
      return
    scenario_name = prompt[(start_idx+1):end_idx]

    for task_type in UserPrompt.valid_task_types:
      if scenario_name.find(task_type) >= 0:
        self.restart_sound_thread(task_type)
        return
    
    rospy.logwarn('Unknown task: ' + scenario_name)
    self.restart_sound_thread('unknown')

  def execute_rumble(self):
    if self.rumble_type is not None:
      self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
      
      if self.rumble_type == 'long':
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=1.0)]))
        t_end = rospy.Time.now()+rospy.Duration(2.0)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
        
      elif self.rumble_type == 'short':
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=1.0)]))
        t_end = rospy.Time.now()+rospy.Duration(1.0)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
        
      elif self.rumble_type == 'shortx3':
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=1.0)]))
        t_end = rospy.Time.now()+rospy.Duration(0.4)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
        if self.rumble_type is None:
          return
        t_end = rospy.Time.now()+rospy.Duration(0.2)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=1.0)]))
        t_end = rospy.Time.now()+rospy.Duration(0.4)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
        if self.rumble_type is None:
          return
        t_end = rospy.Time.now()+rospy.Duration(0.2)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=1.0)]))
        t_end = rospy.Time.now()+rospy.Duration(0.4)
        while rospy.Time.now() < t_end and self.rumble_type is not None:
          rospy.sleep(0.1)
        self.rumble_pub.publish(JoyFeedbackArray([JoyFeedback(type=1, id=0, intensity=0.0)]))
        
      else:
        rospy.logerr('user_prompt > unrecognized rumble_type=%s' % rumble_type)

  def restart_rumble_thread(self, new_rumble_type):
    if self.rumble_thread is not None:
      self.rumble_type = None
      self.rumble_thread.join()
      self.rumble_thread = None
    self.rumble_type = new_rumble_type
    self.rumble_thread = threading.Thread(target=self.execute_rumble)
    self.rumble_thread.start()

  def callback_marshall_mode(self, msg):
    new_mode = msg.data
    if len(self.marshall_mode) > 0:
      if self.marshall_mode != 'rl' and new_mode == 'rl':
        self.restart_rumble_thread('short')
    self.marshall_mode = new_mode
    
  def callback_prompt(self, msg):
    prompt = msg.data
    print '\n\n'
    rospy.loginfo('\n' + prompt)
    if prompt.find('ready to reset_state+apply_controller') >= 0:
      self.accept_prompt = True
      print '- steer to starting pos'
      print '- press Gamepad.START / type \'start\' to continue'
      
      self.announce_task(prompt)
      
      self.restart_rumble_thread('shortx3')
    else:
      self.accept_prompt = False
      
    if prompt.find('finished policy run and ceasing control') >= 0:
      self.restart_rumble_thread('long')
  
  def spin(self):
    while not rospy.is_shutdown():
      try:
        user_input = str(raw_input())
      except KeyboardInterrupt:
        print '\nGot Ctrl-C; shutting down'
        break
      if user_input == "start":
        self.restart_pub.publish()
      
if __name__ == '__main__':
  try:
    user_prmpt = UserPrompt()
    user_prmpt.spin()
  except rospy.ROSInterruptException:
    pass
