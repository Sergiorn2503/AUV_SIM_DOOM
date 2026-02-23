import math
import random
import numpy as np

class CommunicationPhysics:
    def __init__(self):
        # Constants
        self.OPTICAL_RANGE = 100.0  # meters
        self.ACOUSTIC_RANGE = 3000.0  # meters
        self.NOISE_FLOOR = 75.0  # dB
        self.SOURCE_LEVEL_ACOUSTIC = 185.0  # dB re 1 uPa @ 1m
        self.SOURCE_LEVEL_OPTICAL = 100.0  # Arbitrary power units
        self.FREQUENCY_KHZ = 25.0  # Acoustic frequency
        self.SOUND_SPEED = 1500.0  # m/s

    def calculate_thorp_absorption(self, freq_khz):
        """
        Calculates absorption coefficient using Thorp's formula.
        Returns alpha in dB/km.
        """
        f2 = freq_khz ** 2
        alpha = (0.11 * f2 / (1 + f2)) + (44 * f2 / (4100 + f2)) + (2.75e-4 * f2) + 0.003
        return alpha

    def calculate_acoustic_snr(self, distance, depth=0.0):
        """
        Calculates SNR for acoustic link.
        TL = 20*log10(R) + alpha*R*1e-3
        SNR = SL - TL - NL
        
        Depth Penalty: Signal degrades with depth due to thermocline/multipath.
        """
        if distance <= 0:
            return 100.0 # Max SNR at 0 distance

        alpha = self.calculate_thorp_absorption(self.FREQUENCY_KHZ)
        transmission_loss = 20 * math.log10(distance) + (alpha * distance * 1e-3)
        
        # Artificial Depth Penalty REMOVED for robust control
        depth_penalty = 0.0
        
        snr = self.SOURCE_LEVEL_ACOUSTIC - transmission_loss - self.NOISE_FLOOR - depth_penalty
        return max(snr, 0.0)

    def calculate_optical_snr(self, distance):
        """
        Calculates SNR for optical link (Blue Laser).
        Simple exponential decay model.
        """
        if distance <= 0:
            return 100.0
        
        # Attenuation coefficient for clear ocean water (approx 0.05 - 0.1 m^-1)
        attenuation_coeff = 0.08 
        
        # Signal strength decays exponentially: P = P0 * e^(-c*R)
        # We'll map this to a dB-like scale relative to range
        if distance > self.OPTICAL_RANGE:
            return 0.0
            
        signal_strength = self.SOURCE_LEVEL_OPTICAL * math.exp(-attenuation_coeff * distance)
        # Normalize to some SNR-like value, assuming noise floor is low for optical
        snr = 20 * math.log10(signal_strength + 1e-6) # + small epsilon
        
        # Clamp for display
        return max(snr, 0.0)

    def calculate_doppler_shift(self, relative_velocity):
        """
        Calculates Doppler shift.
        delta_f = (v / c) * f
        """
        shift = (relative_velocity / self.SOUND_SPEED) * (self.FREQUENCY_KHZ * 1000)
        return shift

    def check_scintillation(self, position):
        """
        Simulates scintillation effects (random signal drops) based on position.
        Simulates crossing internal waves.
        """
        # Simplified model: Random drops in certain regions or randomly over time
        # Here we just use a random probability for demo purposes
        # In a real sim, this would depend on a map of internal waves
        if random.random() < 0.05: # 5% chance of scintillation
            return True
        return False

    def determine_packet_loss(self, snr):
        """
        Determines if a packet is lost based on SNR.
        """
        # Simple model: Probability of loss increases as SNR decreases
        # SNR > 20 dB: 0% loss
        # SNR < 0 dB: 100% loss
        # Linear interp in between
        if snr >= 20.0:
            prob_loss = 0.0
        elif snr <= 0.0:
            prob_loss = 1.0
        else:
            prob_loss = 1.0 - (snr / 20.0)
            
        return random.random() < prob_loss

    def apply_control_noise(self, value, snr):
        """
        Applies noise to a control value (velocity) based on SNR.
        Lower SNR = Higher Variance.
        """
        if snr <= 0:
            return 0.0 # Signal lost completely
            
        if snr > 30:
            noise_std = 0.0
        else:
            # Noise increases as SNR drops
            noise_std = 0.5 * (1.0 - (snr / 30.0))
            
        noise = random.gauss(0, noise_std)
        return value + noise
