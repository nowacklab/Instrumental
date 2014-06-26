# -*- coding: utf-8 -*-
# Copyright 2014 Chris Rogers, Nate Bogdanowicz
"""
Driver module for Attocube stages. Interfaces with the ECC100 controller,
which has support for various stages:
    * ECS3030
    * ECS3040
    * ECS3050
    * ECS3060
    * ECS3070
    * ECS3080
    * ECS5050
    * ECGp5050
    * ECGt5050
    * ECR3030
"""

from __future__ import print_function
import time
from ctypes import (c_int32, c_bool, byref, create_string_buffer,
                    Structure, POINTER)
from instrumental import Q_

__all__ = ['LinearStage', 'Goniometer', 'RotationStage', 'ECC100']

# Define constants matching the enums in ecc.dll
LINEAR_STAGE, GONIOMETER, ROTATION_STAGE = 0, 1, 2

_err_map = {
    -1: 'Unspecified error.',
    1: 'Communication timeout.',
    2: 'No active connection to device.',
    3: 'Error in communication with driver.',
    7: 'Device is already in use.',
    9: 'Parameter out of range.',
    10: 'Feature only available in pro version.'
}


class EccInfo(Structure):
    _fields_ = [('id', c_int32), ('locked', c_bool)]


class Actor(object):
    def __init__(self, controller, axis):
        self._c = controller
        self.axis = axis
        self.name = self._c._getActorName(self.axis)
        self.type = self._c._getActorType(self.axis)
        self.enable()

    def __repr__(self):
        return "<{} {}, axis={}>".format(self.__class__.__name__, self.name,
                                         self.axis)

    def enable(self):
        """ Enables communication from the controller to the stage. """
        self._c._controlOutput(self.axis, enable=True, set=True)

    def disable(self):
        """ Disables communication from the controller to the stage. """
        self._c._controlOutput(self.axis, enable=False, set=True)

    def is_enabled(self):
        """ Returns whether communication to the stage is enabled. """
        return self._c._controlOutput(self.axis, set=False)

    def is_ref_position_valid(self):
        """ Returns whether the reference position is valid. """
        return self._c._getStatusReference(self.axis)

    def set_amplitude(self, amplitude):
        """ Sets the amplitude of the actuator signal.

        This modifies the step size of the positioner.

        Parameters
        ----------
        amplitude : pint.Quantity
            amplitude of the actuator signal in volt-compatible units. The
            allowed range of inputs is from 0 V to 45 V.
        """
        amp_in_mV = int(Q_(amplitude).to('mV').magnitude)
        if not (0 <= amp_in_mV <= 40e3):
            raise Exception("Amplitude must be between 0 and 45 V")
        self._c._controlAmplitude(self.axis, amp_in_mV, set=True)

    def get_amplitude(self):
        """ Gets the amplitude of the actuator signal. """
        amp_in_mV = self._c._controlAmplitude(self.axis, set=False)
        return Q_(amp_in_mV, 'mV').to('V')

    def set_frequency(self, frequency):
        """ Sets the frequency of the voltage signal applied to the stage. The
        frequency is proportional to the travel speed of the positioner.

        Parameters
        ----------
        frequency : pint.Quantity
            frequency of the actuator signal in Hz-compatible units. The
            allowed range of inputs is from 1 Hz to 2 kHz.
        """
        freq_in_mHz = int(Q_(frequency).to('mHz').magnitude)
        if not (1e3 <= freq_in_mHz <= 2e6):
            raise Exception("Frequency must be between 1 Hz and 2 kHz")
        self._c._controlFrequency(self.axis, freq_in_mHz, set=True)

    def get_frequency(self):
        """ Gets the frequency of the actuator signal. """
        freq_in_mHz = self._c._controlFrequency(self.axis, set=False)
        return Q_(freq_in_mHz, 'mHz').to('Hz')

    def step_once(self, backward=False):
        """ Step once. """
        self._c._setSingleStep(self.axis, backward)

    def start_stepping(self, backward=False):
        """
        Step continously until stopped. This will stop any ongoing motion in
        the opposite direction.
        """
        if backward:
            self._c._controlContinuousBkwd(self.axis, enable=True, set=True)
        else:
            self._c._controlContinuousFwd(self.axis, enable=True, set=True)

    def stop_stepping(self):
        """ Stop any continuous stepping. """
        if self.is_stepping_backward():
            self._c._controlContinuousBkwd(self.axis, enable=False, set=True)
        elif self.is_stepping_forward():
            self._c._controlContinuousFwd(self.axis, enable=False, set=True)

    def is_stepping_backward(self):
        return self._c._controlContinuousBkwd(self.axis, set=False)

    def is_stepping_forward(self):
        return self._c._controlContinuousFwd(self.axis, set=False)

    def get_position(self):
        pos = self._c._getPosition(self.axis)
        return Q_(pos, self._pos_units)

    def get_ref_position(self):
        pos = self._c._getReferencePosition(self.axis)
        return Q_(pos, self._pos_units)

    def move_to(self, pos, wait=False):
        """ Move to a location using closed loop control. """
        pos_mag = int(Q_(pos).to(self._pos_units).magnitude)
        self._c._controlTargetPosition(self.axis, pos_mag, set=True)
        self._c._controlMove(self.axis, enable=True, set=True)

        if wait:
            pass


class LinearStage(Actor):
    def __init__(self, device, axis):
        super(LinearStage, self).__init__(device, axis)
        self._pos_units = 'nm'


class Goniometer(Actor):
    def __init__(self, device, axis):
        super(Goniometer, self).__init__(device, axis)
        self._pos_units = 'udeg'


class RotationStage(Actor):
    pass

_actor_class = {
    0: LinearStage,
    1: Goniometer,
    2: RotationStage
}


class ECC100(object):
    """
    Class for interfacing with the Attocube ECC100 controller. Windows-only.
    """
    def __init__(self, lib_path):
        """ Connects to the attocube controller.
        lib_path: location of the file 'ecc.dll'
        """

        # Load the library 'ecc.dll'
        from ctypes import oledll  # Only import if we try to instantiate
        self._lib = oledll.LoadLibrary(lib_path)

        num, info = self._Check()
        if num < 1:
            raise Exception("No Devices Detected")

        # Attempt to connect to the first device
        self._dev_num = 0
        self._Connect()
        self._load_actors()

    def _handle_err(self, retval, message=None, func=None):
        """ This function prints the error code corresponding to 'retval', as
        returned by functions from the ecc.dll library.
        """
        if retval == 0:
            return
        lines = []

        if message:
            lines.append(message)
        if func:
            lines.append("Error returned by '{}'".format(func))
        lines.append(_err_map[retval])

        raise Exception("\n".join(lines))

    def _load_actors(self):
        self.actors = []
        for cur_axis in range(3):
            if self._getStatusConnected(cur_axis):
                actor_typeid = self._getActorType(cur_axis)
                actor = _actor_class[actor_typeid](self, cur_axis)
                self.actors.append(actor)

    def _Check(self):
        """ Checks for devices and returns their info and how many there are.

        Returns
        -------
        num : int
            number of devices connected
        info : list of EccInfo
            info list. Each EccInfo object has the attributes 'id', and
            'locked'
        """
        info = POINTER(EccInfo)()
        num = self._lib.ECC_Check(byref(info))

        info_list = [info[i] for i in range(num)]
        return num, info_list

    def _Close(self):
        """ Closes the connection to the controller. """
        ret = self._lib.ECC_Close(self._dev_handle)
        self._handle_err(ret, func="Close")

    def _Connect(self):
        """
        Attempts to open a connection to the controller. If successful, sets
        the device handle self._dev_handle. Reads from self._dev_num.
        """
        handle = c_int32()
        ret = self._lib.ECC_Connect(self._dev_num, byref(handle))
        self._handle_err(ret, func="Connect")
        self._dev_handle = handle.value

    def _controlActorSelection(self, axis, actor=0, set=False):
        """
        I HAVE NO IDEA WHAT THIS DOES
        Controls the 'actor' property of a particular axis (ex. ECS3030)

        Parameters
        ----------
        axis : int
            axis of control
        actor : int
            actor id, 0..255
        """
        actor = c_int32(actor)
        ret = self._lib.ECC_controlActorSelection(self._dev_handle, axis,
                                                  byref(actor), set)
        self._handle_err(ret, func="controlActorSelection")
        return actor.value

    def _controlAmplitude(self, axis, amplitude=0, set=False):
        """ Controls the amplitude (in mV) of the specified axis. """
        amplitude = c_int32(int(amplitude))
        ret = self._lib.ECC_controlAmplitude(self._dev_handle, axis,
                                             byref(amplitude), set)
        self._handle_err(ret, func="controlAmplitude")
        return amplitude.value

    def _controlAutoReset(self, axis, enable=False, set=False):
        """ Controls the auto-reset setting. """
        enable = c_int32(enable)
        ret = self._lib.ECC_controlAutoReset(self._dev_handle, axis,
                                             byref(enable), set)
        self._handle_err(ret, func="controlAutoReset")
        return bool(enable.value)

    def _controlContinuousBkwd(self, axis, enable=False, set=False):
        """ Controls continuous backward motion of the specified axis. """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlContinousBkwd(self._dev_handle, axis,
                                                 byref(enable), set)
        self._handle_err(ret, func="controlContinousBkwd")
        return bool(enable.value)

    def _controlContinuousFwd(self, axis, enable=False, set=False):
        """ Controls continuous forward motion of the specified axis. """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlContinousFwd(self._dev_handle, axis,
                                                byref(enable), set)
        self._handle_err(ret, func="controlContinousFwd")
        return bool(enable.value)

    def _controlDeviceId(self, id=0, set=False):
        """ Controls the device identifier stored in the device flash. """
        id = c_int32(id)
        ret = self._lib.ECC_controlDeviceId(self._dev_handle, byref(id), set)
        self._handle_err(ret, func="controlDeviceId")
        return id.value

    def _controlEotOutputDeactivate(self, axis, enable=False, set=False):
        """
        Controls whether the given axis should deactivate its output when end
        of travel (EOT) is reached.
        """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlEotOutputDeactive(self._dev_handle, axis,
                                                     byref(enable), set)
        self._handle_err(ret, func="controlEotOutputDeactivate")
        return bool(enable.value)

    def _controlExtTrigger(self, axis, enable=False, set=False):
        """ Controls the input trigger for steps. """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlExtTrigger(self._dev_handle, axis,
                                              byref(enable), set)
        self._handle_err(ret, func="controlExtTrigger")
        return bool(enable.value)

    def _controlFixOutputVoltage(self, axis, voltage=0, set=False):
        """ Controls the DC level on the output in uV. """
        voltage = c_int32(int(voltage))
        ret = self._lib.ECC_controlFixOutputVoltage(self._dev_handle, axis,
                                                    byref(voltage), set)
        self._handle_err(ret, func="controlFixOutputVoltage")
        return voltage.value

    def _controlFrequency(self, axis, frequency=0, set=False):
        """ Control the frequency parameter.  """
        frequency = c_int32(int(frequency))
        ret = self._lib.ECC_controlFrequency(self._dev_handle, axis,
                                             byref(frequency), set)
        self._handle_err(ret, func="controlFrequency")
        return frequency.value

    def _controlMove(self, axis, enable=False, set=False):
        """
        Controls the feedback-control loop used for positioning the specified
        axis.
        """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlMove(self._dev_handle, axis,
                                        byref(enable), set)
        self._handle_err(ret, func="controlMove")
        return bool(enable.value)

    def _controlOutput(self, axis, enable=False, set=False):
        """ Controls the 'output' state of a specific axis. """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlOutput(self._dev_handle, axis,
                                          byref(enable), set)
        self._handle_err(ret, func="controlOutput")
        return bool(enable.value)

    def _controlReferenceAutoUpdate(self, axis, enable=False, set=False):
        """ Controls the reference auto update setting. """
        enable = c_bool(enable)
        ret = self._lib.ECC_controlReferenceAutoUpdate(self._dev_handle, axis,
                                                       byref(enable), set)
        self._handle_err(ret, func="controlReferenceAutoUpdate")
        return bool(enable.value)

    def _controlTargetPosition(self, axis, target=0, set=False):
        """ Control the target position of the feedback control loop. """
        target = c_int32(int(target))
        ret = self._lib.ECC_controlTargetPosition(self._dev_handle, axis,
                                                  byref(target), set)
        self._handle_err(ret, func="controlTargetPosition")
        return target.value

    def _controlTargetRange(self, axis, range=0, set=False):
        """
        Control the range around the target position where the stage is
        considered to be at the target.
        """
        range = c_int32(int(range))
        ret = self._lib.ECC_controlTargetRange(self._dev_handle, axis,
                                               byref(range), set)
        self._handle_err(ret, func="controlTargetRange")
        return range.value

    def _getActorName(self, axis):
        """ Returns the name of the 'actor' of the specified axis. """
        buf = create_string_buffer(20)
        ret = self._lib.ECC_getActorName(self._dev_handle, axis, buf)
        self._handle_err(ret, func="getActorName")
        return buf.value.strip()

    def _getActorType(self, axis):
        """ Returns  an int corrsesponding to the type of actor associated with
        the specified axis.
        """
        _type = c_int32()
        ret = self._lib.ECC_getActorType(self._dev_handle, axis,
                                         byref(_type))
        self._handle_err(ret, func="getActorType")
        return _type.value

    def _getDeviceInfo(self):
        """
        Returns the device ID and a boolean that indicates whether or not the
        device is locked.
        """
        dev_id = c_int32(0)
        locked = c_bool(0)
        ret = self._lib.ECC_getDeviceInfo(self._dev_num, byref(dev_id),
                                          byref(locked))
        self._handle_err(ret, func="getDeviceInfo")
        return dev_id.value, locked.value

    def _getPosition(self, axis):
        """
        Returns the position of the stage on the specifed axis (in nm for
        linear stages, micro radians for goniometers).
        """
        position = c_int32()
        ret = self._lib.ECC_getPosition(self._dev_handle, axis,
                                        byref(position))
        self._handle_err(ret, func="getPosition")
        return position.value

    def _getReferencePosition(self, axis):
        """
        Returns the reference position of the stage on the specifed axis (in nm
        for linear stages, micro radians for goniometers).
        """
        reference = c_int32()
        ret = self._lib.ECC_getReferencePosition(self._dev_handle, axis,
                                                 byref(reference))
        self._handle_err(ret, func="getReferencePosition")
        return reference.value

    def _getStatusConnected(self, axis):
        """ Returns whether actor given by `axis` is connected or not. """
        connected = c_int32()
        ret = self._lib.ECC_getStatusConnected(self._dev_handle, axis,
                                               byref(connected))
        self._handle_err(ret)
        return bool(connected.value)

    def _getStatusEotBkwd(self, axis):
        """
        Returns whether the given axis is at the end of travel (EOT) in the
        backward direction.
        """
        at_eot = c_bool()
        ret = self._lib.ECC_getStatusEotBkwd(self._dev_handle, axis,
                                             byref(at_eot))
        self._handle_err(ret, func="getStatusEotBkwd")
        return bool(at_eot.value)

    def _getStatusEotFwd(self, axis):
        """
        Returns whether the given axis is at the end of travel (EOT) in the
        forward direction.
        """
        at_eot = c_bool()
        ret = self._lib.ECC_getStatusEotFwd(self._dev_handle, axis,
                                            byref(at_eot))
        self._handle_err(ret, func="getStatusEotFwd")
        return bool(at_eot.value)

    def _getStatusError(self, axis):
        """ Returns True if there is an error due to sensor malfunction. """
        has_err = c_bool()
        ret = self._lib.ECC_getStatusError(self._dev_handle, axis,
                                           byref(has_err))
        self._handle_err(ret, func="getStatusError")
        return bool(has_err.value)

    def _getStatusFlash(self, axis):
        """ Returns whether the flash is being written to or not. """
        flash_is_writing = c_bool()
        ret = self._lib.ECC_getStatusFlash(self._dev_handle, axis,
                                           byref(flash_is_writing))
        self._handle_err(ret, func="getStatusFlash")
        return bool(flash_is_writing.value)

    def _getStatusMoving(self, axis):
        """
        Returns whether the specified axis is idle, moving or pending.

        Returns
        -------
        int
            0: Idle, 1: Moving, 2: Pending
        """
        moving = c_int32()
        ret = self._lib.ECC_getStatusMoving(self._dev_handle, axis,
                                            byref(moving))
        self._handle_err(ret, func="getStatusMoving")
        return moving.value

    def _getStatusReference(self, axis):
        """ Checks whether or not the reference position is valid. """
        status = c_bool()
        ret = self._lib.ECC_getStatusReference(self._dev_handle, axis,
                                               byref(status))
        self._handle_err(ret, func="getStatusReference")
        return bool(status.value)

    def _getStatusTargetRange(self, axis):
        """ Returns whether the stage is considered to be at its target. """
        at_target = c_bool()
        ret = self._lib.ECC_getStatusTargetRange(self._dev_handle, axis,
                                                 byref(at_target))
        self._handle_err(ret, func="getStatusTargetRange")
        return bool(at_target.value)

    def _setReset(self, axis):
        """ Resets the reference position to the current position.  """
        ret = self._lib.ECC_setReset(self._dev_handle, axis)
        self._handle_err(ret, func="resetReference")

    def _setSingleStep(self, axis, backward):
        """ Causes the stage along the specified axis to take a single 'step'

        direction = 0 -> positive movement for linear stages, and negative
        movement for goniometers

        direction = 1 -> negative movement for linear stages, and positive
        movement for goniometers
        """
        ret = self._lib.ECC_setSingleStep(self._dev_handle, axis, backward)
        self._handle_err(ret, func="setSingleStep")

    # As-yet unconverted functions below here

    def set_actor(self, axis, actor):
        """ Sets the 'actor' property of the specified axis

        Parameters
        ----------
        actor:  int
            id corresponding to a particular `actor`
        """
        self._controlActorSelection(axis, actor, set=True)

    def get_actor(self, axis):
        """
        Returns the 'actor' property (as an integer) of the specified axis
        """
        actor = self._controlActorSelection(axis, set=False)
        return actor

    def setContinuous(self, axis, direction, control):
        """ Allows for control of continuous movement along the specified axis

        direction = 0 -> movement in the positive direction for
        linear stages, and in the negative direction for goniometers

        direction = 1 -> movement in the negative direction for
        linear stages, and in the positive direction for goniometers

        control = 1 -> start motion in the specified direction

        control = 0 -> stop all continuous motion on the specified axis
        """
        if direction == 0:
            self.controlContinuousFwd(axis, control, set=True)
        if direction == 1:
            self.controlContinuousBkwd(axis, control, set=True)

    def continuousMove(self, axis, direction, duration):
        """ Initiates continuous motion along the specified axis for the
        specified duration

        direction: a value of zero corresponds to the positive direction for
        linear stages and the negative direction for goniometers

        duration: duration of motion, in seconds
        """
        self.setContinuous(axis, direction, control=1)
        time.sleep(duration)
        self.setContinuous(axis, direction, control=0)

    def setMove(self, axis, enable):
        """ Turn on and off feedback-control loop for positioning the
        specified axis

        enable: a  value of 0 corresponds to OFF and a value of 1 to ON
        """
        self.controlMove(axis, enable, set=True)

    def getMove(self, axis):
        """ Returns whether or not the feedback-control loop is ON (returns 1)
        or OFF (returns 0)
        """
        enable = self.controlMove(axis, set=False)
        return enable

    def setTargetPosition(self, axis, target):
        """ Set the target position of the feedback control loop for the
        specified axis

        target: target position, in nm for linear stages, in micro radians for
        goniometers
        """
        self.controlTargetPosition(axis, target, set=True)

    def getTargetPosition(self, axis):
        """ Returns the target position of the feedback control loop for the
        specified axis, in nm for linear stages and in micro-radians for
        goniometers
        """
        # Target Position in nm or micro radians
        targetPosition = self.controlTargetPosition(axis, set=False)
        return targetPosition

    def moveTo(self, axis, target):
        """ (For Linear Stages Only)

        Uses the feedback-control loop to move to and stablilize the position
        at the target

        target: target position, in nm for linear stages, in micro radians for
        goniometers
        """
        self.setTargetPosition(axis, target)
        self.setMove(axis, enable=True)

    def endMove(self, axis):
        """ Turns off the feedback-control loop for the specified axis. """
        self.setMove(axis, enable=False)

    def setReference(self, axis):
        """ This method finds the reference position, sets it to zero, and then
        moves the stage to zero (all along the specified axis).
        If the reference position is already valid, it is reset to zero, and
        the stage is then moved to zero

        For linear stages, the feedback stabilized control loop is used, and
        when the method exits, the feedback control loop is left on.  At this
        point, both the reference position and the position itself should be
        within 1nm of zero.

        For goniometers, there is no feedback stabilization, so when the method
        exits, the axis specified is no longer being actively driven.  At this
        point, the reference position should be within 2.5 milli radians of
        zero, and the actual position should be within 2.5 milli radians of the
        reference position.

        Note that depending upon whether the reference position is initally
        valid, and where the stage is, the method may move the stage all the
        way to EOT (end of travel).
        """
        stageType = self.getActorType(axis)
        if stageType == LINEAR_STAGE:
            self.linearReference(axis)
        elif stageType == GONIOMETER:
            self.gonioReference(axis)

    def gonioReference(self, axis, deltaT=0.1):
        """ This method finds a valid reference position, resets that reference
        position to zero, and then moves the stage to the reference position

        This method is designed for goniometer stages
        """
        refStatus = self.getStatusReference(axis)
        self.setFrequency(axis, frequency=1000000)
        # If the reference status is not valid:
        if not(refStatus):
            # move back and forth once around the current position
            self.moveAroundFixedTime(axis, time=2)
            refStatus = self.getStatusReference(axis)

            # Set the direction in which to search for the reference position
            position = self.getPosition(axis)
            if position > 0:
                direction = 1
            else:
                direction = 0

            # If the reference position is still invalid:
            while not(refStatus):
                # Move the stage continuously in one direction
                self.setContinuous(axis, direction, control=1)
                moving = True

                # Keep waiting as long as the stage is still moving and the
                # reference position has not been found
                while moving & (not(refStatus)):
                    p0 = self.getPosition(axis)
                    time.sleep(deltaT)
                    p = self.getPosition(axis)
                    if abs(p-p0) < 10000:
                        moving = False

                    refStatus = self.getStatusReference(axis)

                # If the reference position has not been found, but the stage
                # has reached EOT, switch the search direction
                direction = not(direction)

            # By this point, the reference should have been found

        # Stop the continuous motion, then get the reference position
        # and move the stage to the reference position
        self.setContinuous(axis, direction=0, control=0)
        time.sleep(deltaT)
        refPos = self.getReferencePosition(axis)
        self.gonioMoveTo(axis, refPos)

        # Reset the reference position to the current position (near zero),
        # and then move back and forth over it to make it valid once more
        self.resetReference(axis)
        self.moveAroundFixedTime(axis, time=0.2)

        # Move the stage to the reference postion
        refPos = self.getReferencePosition(axis)
        self.gonioMoveTo(axis, refPos)

    def linearReference(self, axis, deltaT=0.1):
        """ This method finds a valid reference position, resets that reference
        position to zero, and then moves the stage to the reference position

        This method is designed for linear stages
        """
        refStatus = self.getStatusReference(axis)
        self.setFrequency(axis, frequency=1000000)

        # If the reference status is not valid:
        if not(refStatus):
            # Move back and forth around the current position to try to get a
            # valid reference position
            position = self.getPosition(axis)
            self.moveAround(axis, position, distance=1000000)
            refStatus = self.getStatusReference(axis)

            # Set the direction in which to search for a valid reference pos
            if position > 0:
                maxDist = -25000000
            else:
                maxDist = 25000000

            # If the reference position is still invalid:
            while not(refStatus):
                # Set movement to a target past the end of travel
                self.moveTo(axis, target=maxDist)
                moving = True

                # Keep waiting as long as the stage is moving and the reference
                # position is invalid
                while moving & (not(refStatus)):
                    p0 = self.getPosition(axis)
                    time.sleep(deltaT)
                    p = self.getPosition(axis)
                    if abs(p-p0) < 1000:
                        moving = False

                    refStatus = self.getStatusReference(axis)

                # If EOT has been reached and no reference has yet been found,
                # switch the search direction
                maxDist = -maxDist
                refStatus = self.getStatusReference(axis)

        # By this point a valid reference should have been found

        # Stop movement and move to the (valid) reference position
        self.endMove(axis)
        time.sleep(deltaT)
        refPos = self.getReferencePosition(axis)
        self.moveTo(axis, refPos)
        self.waitUntilStopped(axis)

        # Reset the reference postion to the current position and then
        # move back and forth to make the reference valid again
        self.resetReference(axis)
        self.moveAround(axis, centrePos=0, distance=10000)

        # Move the stage to zero
        self.moveTo(axis, target=0)

    def gonioMoveTo(self, axis, target):
        """ This method moves a goniometer stage to the target position

        target: target position in nm
        """
        self.setAmplitude(axis, 30000)
        self.setFrequency(axis, 1000000)
        # On these settings the gonio moves ~ 4 100 000 urad/s
        speed = 4100000.

        # Find delta, the difference between the target position and the
        # current postion
        position = self.getPosition(axis)
        delta = target - position

        # While delta is larger than 40 000 micro radians (~ 10 single steps),
        # use timed continuous motion to move towards the target position
        while abs(delta) > 40000:
            if delta > 0:
                direction = 1
            else:
                direction = 0
            self.continuousMove(axis, direction, duration=abs(delta/speed))
            time.sleep(0.05)

            # Update delta
            position = self.getPosition(axis)
            delta = target - position

        # While distance between the target and the actual position is greater
        # than 2500 micro-radians, move towards the target using single steps
        while abs(delta) > 2500:
            if delta > 0:
                direction = 1
            else:
                direction = 0
            self.setSingleStep(axis, direction)
            time.sleep(0.05)
            position = self.getPosition(axis)
            delta = target - position

    def moveAroundFixedTime(self, axis, time):
        """ Moves the stage back and forth once around the initial position.
        The total movement time is 4 times the 'time' parameter (in seconds).

        This method uses continuous motion, and so works for all types of
        stages.
        """
        self.continuousMove(axis, direction=1, duration=time)
        self.continuousMove(axis, direction=0, duration=2*time)
        self.continuousMove(axis, direction=1, duration=time)

    def moveAround(self, axis, centrePos, distance):
        """ This method moves the stage to the centre position, and then once
        back and forth around that position, moving a distance specified
        by the parameter 'distance' in either direction

        centrePos: Centre Position of the motion, in nm

        distance: distance travelled to on either side of the centre position,
        in nm

        Note that this method uses the feedback stabilize 'moveTo' method, and
        so only works for linear stages
        """
        self.moveTo(axis, centrePos)
        self.waitUntilStopped(axis)
        self.moveTo(axis, centrePos + distance)
        self.waitUntilStopped(axis)
        self.moveTo(axis, centrePos-distance)
        self.waitUntilStopped(axis)
        self.moveTo(axis, centrePos)
        self.waitUntilStopped(axis)

    def waitUntilStopped(self, axis, deltaT=0.05, deltaPos=1000):
        """ This function does not return until the stage is no longer moving
        more than the amount deltaPos in time interval deltaT.  It is designed
        primarily for use with linear stages and detecting when the 'moveTo'
        method has reached its target.  Given the default values of deltaT and
        deltaPos, it will also detect when end of travel has been reached.

        Note that if the frequency is less then ~ 100 Hz, then deltaPos
        may need to be reduced (although if it is reduced too far, then EOT
        will not be properly detected)
        """
        moving = True

        # While the stage is still moving, keep waiting to return
        while moving:
            p0 = self.getPosition(axis)
            time.sleep(deltaT)
            p = self.getPosition(axis)
            if abs(p-p0) < deltaPos:
                moving = False
            time.sleep(deltaT)
        return
