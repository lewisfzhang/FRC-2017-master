package com.team254.frc2017.auto.actions;

public class PrintDebugAction extends RunOnceAction implements Action {
    String debugMessage;

    public PrintDebugAction(String s) {
        debugMessage = s;
    }

    @Override
    public void runOnce() {
        System.out.println(debugMessage);
    }

}
