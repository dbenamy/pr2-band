#!/usr/bin/env python
import sys
from time import sleep
from random import randint
import subprocess

import roslib; roslib.load_manifest('conductor')
import rospy
from rospy import Duration
from conductor.msg import Song, Note
from std_msgs.msg import String

from midi.MidiOutStream import MidiOutStream
from midi.MidiInFile import MidiInFile
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

try:
  import Tkinter
  import tkSnack
except:
  def init_tk():
    pass


class MidiSongBuilder(MidiOutStream):
  def __init__(self, config):
    MidiOutStream.__init__(self)
    self.config = config
    self.song = []
    self.current = None

  def note_on(self, channel=None, note=None, velocity=None):
    if channel in self.config['channels']:
      if self.current is not None:
        print 'cutting note short for new one'
        self.note_off(channel, self.current[0], 100)
#      print 'on', channel, note, velocity, self.abs_time()
      self.current = (note, self.abs_time())
 
  def note_off(self, channel=None, note=None, velocity=None):
    if channel in self.config['channels']:
      if self.current is None or self.current[0] != note:
        print "Ignoring off %s because it's not on" % note
        return
      if note not in self.config['keys']:
        print "Ignoring note %s because it's not in CONFIG"
        return
#      print 'off', channel, note, velocity, self.abs_time()
      length = self.abs_time() - self.current[1]
      start = self.current[1]
      end = self.abs_time() 
#      print 'adding note %s of length %s from %s to %s' % (note, length,
#                                                           start, end)
      self.song.append((float(start) / SCALE, note, float(length) / SCALE))
      self.current = None


def strip_leading_silence(song):
  silence = song[0][0]
  print 'stripping %s secs silence from start' % silence
  return [(n[0] - silence, n[1], n[2]) for n in song]


def play_freq(freq, duration):
    """play a note of freq (hertz) for duration (seconds)"""
    global snd
    filt = tkSnack.Filter('generator', freq, 30000, 0.0, 'sine', int(11500*duration))
    snd.play(filter=filt, blocking=1)

 
def play_note(key, length):
  f = (2 ** ((float(key) - 69) / 12)) * 440
  play_freq(f, length)


def play_song(song):
  time = 0
  for note in song:
    if time < note[0]:
      print 'sleeping for %.3f until %.3f' % (note[0] - time, note[0])
      sleep(note[0] - time)
    print '%.3f: playing key %s for length %.3f' % (note[0], note[1], note[2])
    play_note(note[1], note[2])
    time = note[0] + note[2]


def init_tk():
  root = Tkinter.Tk()
  tkSnack.initializeSnack(root)
  tkSnack.audio.play_gain(100)
  global snd
  snd = tkSnack.Sound()


def make_song(config, midi_file):
  song_builder = MidiSongBuilder(config)
  MidiInFile(song_builder, midi_file).read()
  print ''
  song = song_builder.song
  song.sort(key=lambda n: n[0])
  print song, '\n'
#  song = strip_leading_silence(song)
#  print song, '\n'

  rs = Song()
  for note in song:
    key = str(config['keys'][note[1]])
    rs.song.append(Note(time=Duration(note[0]), key=key))
  return rs

X = 2

def make_mary_song(t):
  s = Song(start_sec=t)
  x = X
  s.song.append(Note(time=Duration(0 * x), key='64'))
  s.song.append(Note(time=Duration(1 * x), key='62'))
  s.song.append(Note(time=Duration(2 * x), key='60'))
  s.song.append(Note(time=Duration(3 * x), key='62'))
  s.song.append(Note(time=Duration(4 * x), key='64'))
  s.song.append(Note(time=Duration(5 * x), key='64'))
  s.song.append(Note(time=Duration(6 * x), key='64'))
  s.song.append(Note(time=Duration(8 * x), key='62'))
  s.song.append(Note(time=Duration(9 * x), key='62'))
  s.song.append(Note(time=Duration(10 * x), key='62'))
  s.song.append(Note(time=Duration(12 * x), key='64'))
  s.song.append(Note(time=Duration(13 * x), key='67'))
  s.song.append(Note(time=Duration(14 * x), key='67'))
  s.song.append(Note(time=Duration(16 * x), key='64'))
  s.song.append(Note(time=Duration(17 * x), key='62'))
  s.song.append(Note(time=Duration(18 * x), key='60'))
  s.song.append(Note(time=Duration(19 * x), key='62'))
  s.song.append(Note(time=Duration(20 * x), key='64'))
  s.song.append(Note(time=Duration(21 * x), key='64'))
  s.song.append(Note(time=Duration(22 * x), key='64'))
  s.song.append(Note(time=Duration(23 * x), key='64'))
  s.song.append(Note(time=Duration(24 * x), key='62'))
  s.song.append(Note(time=Duration(25 * x), key='62'))
  s.song.append(Note(time=Duration(26 * x), key='64'))
  s.song.append(Note(time=Duration(27 * x), key='62'))
  s.song.append(Note(time=Duration(28 * x), key='60'))
  return s

def make_rand_song(t, beats):
  s = Song(start_sec=t)
  for x in range(beats):
    s.song.append(Note(time=Duration(x * X), key=str(randint(40, 70))))
  return s

def make_drum_song(t, measures):
  s = Song(start_sec=t)
  for x in range(measures):
    s.song.append(Note(time=Duration((x * 4 + 0) * X), key='hi-hat'))
    s.song.append(Note(time=Duration((x * 4 + 1) * X), key='hi-hat'))
    s.song.append(Note(time=Duration((x * 4 + 2) * X), key='hi-hat'))
    s.song.append(Note(time=Duration((x * 4 + 3) * X), key='snare'))
  return s

def play_sound():
  soundhandle = SoundClient()
  s = "Thank you"
  print 'Saying: %s'%s
  soundhandle.say(s)


def do_song(midi_file, config_file, piano, drum):
  execfile(config_file, globals(),locals())
  drum_song = make_song(DRUM_CONFIG, midi_file)
  piano_song = make_song(PIANO_CONFIG, midi_file)
  t = rospy.get_time() + 2
  drum_song.start_sec = t
  drum.publish(drum_song)
  piano_song.start_sec = t
  piano.publish(piano_song)

def main():
  #global SCALE
  rospy.init_node('midi_player')
  drum = rospy.Publisher('song_drum', Song)
  piano = rospy.Publisher('song_piano', Song)
  piano_util = rospy.Publisher('piano_util', String)
  sleep(1)

  mode = sys.argv[1]
  if mode == 'ros':
    midi_file = sys.argv[2]
    config_file = sys.argv[3]
    do_song(midi_file, config_file, piano, drum)
  elif mode == 'play':
    init_tk()
    piano_song = make_song(PIANO_CONFIG, midi_file)
    play_song(piano_song)
  elif mode == 'test':
    t = rospy.get_time() + 2
    drum.publish(make_drum_song(t, 8))
#    piano.publish(make_mary_song(t))
    piano.publish(make_rand_song(t, 32))
  elif mode == 'thankyou':
    play_sound()
  elif mode == 'demo':
    piano_util.publish(String('arms_up'))
    raw_input('song 1')
    midi_file = 'songs/With_Or_Without_You.mid'
    config_file = 'config/With_Or_Without_You.cfg'
    do_song(midi_file, config_file, piano, drum)
    raw_input('song 2')
    midi_file = 'songs/here_comes_the_sun_b.mid'
    config_file = 'config/here_comes_the_sun_b.cfg'
    do_song(midi_file, config_file, piano, drum)
    raw_input('arms down- turn on mic and do applause while happening')
    piano_util.publish(String('arms_down'))
    raw_input('thank you')
    subprocess.call(['rosrun', 'sound_play', 'say.py', "thank you very much"])
  else:
    piano_util.publish(String(mode))
  
#  root.withdraw()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
