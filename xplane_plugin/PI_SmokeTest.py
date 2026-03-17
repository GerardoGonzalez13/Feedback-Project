from XPPython3 import xp

class PythonInterface:
    def XPluginStart(self):
        self.Name = "SmokeTest"
        self.Sig  = "team.smoketest"
        self.Desc = "Confirms XPPython3 is running."
        xp.sys_log("[SmokeTest] START")
        return self.Name, self.Sig, self.Desc

    def XPluginEnable(self):
        xp.sys_log("[SmokeTest] ENABLED")
        return 1

    def XPluginDisable(self):
        xp.sys_log("[SmokeTest] DISABLED")

    def XPluginStop(self):
        xp.sys_log("[SmokeTest] STOPPED")

    def XPluginReceiveMessage(self, inFromWho, inMessage, inParam):
        pass