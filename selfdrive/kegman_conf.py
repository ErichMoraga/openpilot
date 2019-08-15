import json
import os

class kegman_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['timeConstant'] == "-1":
      self.conf['timeConstant'] = str(round(CP.lateralTuning.indi.timeConstant,3))
      write_conf = True
    if self.conf['actuatorEffectiveness'] == "-1":
      self.conf['actuatorEffectiveness'] = str(round(CP.lateralTuning.indi.actuatorEffectiveness,3))
      write_conf = True
    if self.conf['outerLoopGain'] == "-1":
      self.conf['outerLoopGain'] = str(round(CP.lateralTuning.indi.outerLoopGain,3))
      write_conf = True
    if self.conf['innerLoopGain'] == "-1":
      self.conf['innerLoopGain'] = str(round(CP.lateralTuning.indi.innerLoopGain,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/zorrotune.json'):
      with open('/data/zorrotune.json', 'r') as f:
        self.config = json.load(f)

      if "timeConstant" not in self.config:
        self.config.update({"timeConstant":"-1"})
        self.element_updated = True

      if "actuatorEffectiveness" not in self.config:
        self.config.update({"actuatorEffectiveness":"-1"})
        self.element_updated = True

      if "outerLoopGain" not in self.config:
        self.config.update({"outerLoopGain":"-1"})
        self.element_updated = True

      if "innerLoopGain" not in self.config:
        self.config.update({"innerLoopGain":"-1"})
        self.element_updated = True


      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"timeConstant":"-1", "actuatorEffectiveness":"-1", "outerLoopGain":"-1", "innerLoopGain":"-1",}

      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    with open('/data/zorrotune.json', 'w') as f:
      json.dump(self.config, f, indent=2, sort_keys=True)
      os.chmod("/data/zorrotune.json", 0o764)
