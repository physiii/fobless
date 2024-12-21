import React, { useEffect, useState, useRef } from 'react';
import {
  View,
  Text,
  PermissionsAndroid,
  Platform,
  Alert,
  StyleSheet,
} from 'react-native';
import { BleManager, Device, State } from 'react-native-ble-plx';
import { Buffer } from 'buffer';
import { 
  Provider as PaperProvider, 
  Appbar, 
  Switch,
  ActivityIndicator,
  MD3DarkTheme,
  Button
} from 'react-native-paper';
import MaterialCommunityIcons from 'react-native-vector-icons/MaterialCommunityIcons';
import { SafeAreaProvider } from 'react-native-safe-area-context';

const manager = new BleManager();

const SERVICE_UUID = '0000abf0-0000-1000-8000-00805f9b34fb';
const CHARACTERISTIC_UUID = '0000abf3-0000-1000-8000-00805f9b34fb';
const TARGET_DEVICE_NAME = "ESP_SPP_SERVER";

const POWER_ON_CODE = 1;
const POWER_OFF_CODE = 0;
const LOCK_CODE = 3;
const UNLOCK_CODE = 5;
const TRUNK_CODE = 9;
const HORN_CODE = 17;

const theme = {
  ...MD3DarkTheme,
  colors: {
    ...MD3DarkTheme.colors,
    primary: '#bb86fc',
  },
};

const App: React.FC = () => {
  const [scannedDevice, setScannedDevice] = useState<Device | null>(null);
  const [connectedDevice, setConnectedDevice] = useState<Device | null>(null);

  const [isScanning, setIsScanning] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [isSending, setIsSending] = useState(false);

  const [connectOn, setConnectOn] = useState(false);
  const [powerOn, setPowerOn] = useState(true);
  const [isLocked, setIsLocked] = useState(false);

  // Keep track if we have already set the state listener
  const stateListenerSet = useRef(false);

  useEffect(() => {
    async function requestPermissions() {
      let allGranted = true;
      if (Platform.OS === 'android') {
        if (Platform.Version >= 31) {
          const result = await PermissionsAndroid.requestMultiple([
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
          ]);
          allGranted = !Object.values(result).some(r => r !== 'granted');
        } else {
          const fineLocation = await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION
          );
          allGranted = fineLocation === 'granted';
        }
      }

      // Auto-connect if permissions are granted
      if (allGranted) {
        console.log('Permissions granted, turning connectOn = true and starting scan');
        setConnectOn(true);
        startScan();
      } else {
        Alert.alert('Permissions required', 'Need all BLE and location permissions granted.');
      }
    }

    requestPermissions();

    // Set up the BLE state listener only once
    if (!stateListenerSet.current) {
      stateListenerSet.current = true;
      manager.onStateChange((state) => {
        console.log('Bluetooth state changed:', state);
        if (state === State.PoweredOn && connectOn && !connectedDevice) {
          console.log('Bluetooth PoweredOn and connectOn = true, starting scan');
          startScan();
        }
      }, true);
    }

    return () => {
      manager.stopDeviceScan();
    };
  }, [connectedDevice, connectOn]);

  const startScan = () => {
    if (!connectOn) {
      console.log('Cannot start scan because connectOn = false');
      return;
    }

    if (connectedDevice) {
      console.log('Already connected to a device, no need to scan');
      return;
    }

    if (isScanning) {
      console.log('Already scanning, stopping then restarting');
      manager.stopDeviceScan();
      setIsScanning(false);
    }

    console.log('Starting scan...');
    setScannedDevice(null);
    setIsScanning(true);

    manager.startDeviceScan(null, null, async (error, device) => {
      if (error) {
        console.warn('Scan error:', error);
        setIsScanning(false);
        setConnectOn(false);
        return;
      }

      if (device && device.name === TARGET_DEVICE_NAME) {
        console.log('Found device:', device.name, 'Stopping scan and connecting');
        manager.stopDeviceScan();
        setIsScanning(false);
        setScannedDevice(device);
        await connectToDevice(device);
      }
    });
  };

  const connectToDevice = async (device: Device) => {
    if (!connectOn) {
      console.log('connectOn is false, not connecting');
      return;
    }
    console.log('Connecting to device:', device.name);
    setIsConnecting(true);
    try {
      const d = await device.connect();
      console.log('Connected to:', d.name);
      await d.discoverAllServicesAndCharacteristics();
      setConnectedDevice(d);

      // Automatically power on
      await sendCommand(POWER_ON_CODE, false);
      setPowerOn(true);

      manager.onDeviceDisconnected(d.id, () => {
        console.log('Device disconnected');
        setConnectedDevice(null);
        // If connectOn is still true, the onStateChange listener should handle reconnection attempts when BLE is PoweredOn.
      });
    } catch (error) {
      console.error('Connection error:', error);
      // Connection failed, turn off connectOn so user can retry
      setConnectOn(false);
    } finally {
      setIsConnecting(false);
    }
  };

  const disconnect = async () => {
    if (!connectedDevice) return;
    console.log('Disconnecting from device');
    setIsConnecting(true);
    try {
      await connectedDevice.cancelConnection();
      setConnectedDevice(null);
      setConnectOn(false);
    } catch (err) {
      console.error('Failed to disconnect:', err);
    } finally {
      setIsConnecting(false);
    }
  };

  const sendCommand = async (command: number, showAlerts: boolean = true) => {
    if (!connectedDevice) {
      if (showAlerts) {
        Alert.alert('Not connected', 'Please connect first.');
      }
      return;
    }
    if (!powerOn && command !== POWER_OFF_CODE && command !== POWER_ON_CODE) {
      if (showAlerts) {
        Alert.alert('No Power', 'Power is off. Turn power on first.');
      }
      return;
    }
    setIsSending(true);
    try {
      await connectedDevice.writeCharacteristicWithResponseForService(
        SERVICE_UUID,
        CHARACTERISTIC_UUID,
        Buffer.from([command]).toString('base64')
      );
      console.log('Command sent:', command);
    } catch (error) {
      console.error('Write error:', error);
      if (showAlerts) {
        Alert.alert('Error', 'Failed to write command.');
      }
    } finally {
      setIsSending(false);
    }
  };

  const handleConnectToggle = (newVal: boolean) => {
    console.log('Connect toggle changed to:', newVal);
    setConnectOn(newVal);
    if (newVal) {
      startScan();
    } else {
      disconnect();
    }
  };

  const handlePowerToggle = async (newVal: boolean) => {
    console.log('Power toggle changed to:', newVal);
    setPowerOn(newVal);
    await sendCommand(newVal ? POWER_ON_CODE : POWER_OFF_CODE);
  };

  const makeMomentaryHandlers = (downCode: number, onCompleted?: () => void) => ({
    onPressIn: () => sendCommand(downCode),
    onPressOut: async () => {
      await sendCommand(POWER_ON_CODE);
      if (onCompleted) onCompleted();
    }
  });

  const lockUnlockHandlers = () => {
    return makeMomentaryHandlers(isLocked ? UNLOCK_CODE : LOCK_CODE, () => {
      setIsLocked(!isLocked);
    });
  };

  const isBusy = isConnecting || isSending;
  const canInteract = !!connectedDevice && !isBusy;
  const isConnected = !!connectedDevice;

  let statusText = '';
  if (!isConnected && !isScanning && !scannedDevice && connectOn) {
    statusText = 'Connecting to device...';
  } else if (isScanning) {
    statusText = 'Scanning for device...';
  } else if (scannedDevice && !isConnected) {
    statusText = `Found ${scannedDevice.name}, connecting...`;
  } else if (!connectOn && !isConnected) {
    statusText = 'Toggle Connect to scan for the device.';
  }

  return (
    <SafeAreaProvider>
      <PaperProvider theme={theme}>
        <Appbar.Header>
          <Appbar.Content 
            title="FOBless" 
            subtitle={isConnected ? `Connected to: ${connectedDevice?.name}` : undefined}
          />
        </Appbar.Header>

        <View style={styles.container}>
          {isBusy && (
            <View style={styles.overlay}>
              <ActivityIndicator animating={true} size="large" color="#bb86fc" />
            </View>
          )}

          {/* Connection Toggle */}
          <View style={styles.row}>
            <MaterialCommunityIcons
              name={connectOn ? "bluetooth" : "bluetooth-off"}
              size={24}
              color="#fff"
              style={{ marginRight: 5 }}
            />
            <Text style={styles.label}>Connect</Text>
            <Switch 
              value={connectOn}
              onValueChange={handleConnectToggle}
              disabled={isBusy}
            />
          </View>

          {/* Power Toggle */}
          {isConnected && (
            <View style={styles.row}>
              <MaterialCommunityIcons
                name="power"
                size={24}
                color="#fff"
                style={{ marginRight: 5 }}
              />
              <Text style={styles.label}>Power On/Off</Text>
              <Switch 
                value={powerOn}
                onValueChange={handlePowerToggle}
                disabled={!canInteract}
              />
            </View>
          )}

          {/* Control Section */}
          {isConnected && (
            <View style={styles.section}>
              <Text style={styles.sectionTitle}>Controls</Text>

              <View style={styles.buttonRow}>
                <Button
                  mode="contained"
                  style={styles.actionButton}
                  icon={isLocked ? "lock" : "lock-open"}
                  {...lockUnlockHandlers()}
                  disabled={!canInteract || !powerOn}
                >
                  {isLocked ? "Unlock" : "Lock"}
                </Button>

                <Button
                  mode="contained"
                  style={styles.actionButton}
                  icon="car-door"
                  {...makeMomentaryHandlers(TRUNK_CODE)}
                  disabled={!canInteract || !powerOn}
                >
                  Trunk
                </Button>
              </View>

              <View style={styles.buttonRow}>
                <Button
                  mode="contained"
                  style={styles.actionButton}
                  icon="volume-high"
                  {...makeMomentaryHandlers(HORN_CODE)}
                  disabled={!canInteract || !powerOn}
                >
                  Horn
                </Button>
              </View>
            </View>
          )}

          {/* Status Messages */}
          {!isConnected && (
            <Text style={styles.infoText}>
              {statusText}
            </Text>
          )}
        </View>
      </PaperProvider>
    </SafeAreaProvider>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 20,
    backgroundColor: '#121212'
  },
  infoText: {
    fontSize: 16,
    textAlign: 'center',
    marginVertical: 15,
    color: '#fff',
  },
  row: {
    flexDirection: 'row',
    alignItems: 'center',
    marginVertical: 20,
  },
  label: {
    fontSize: 16,
    marginRight: 10,
    color: '#fff'
  },
  section: {
    marginVertical: 20
  },
  sectionTitle: {
    fontSize: 18,
    color: '#fff',
    marginBottom: 10,
    textAlign: 'center',
    fontWeight: 'bold'
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'center',
    flexWrap: 'wrap',
  },
  actionButton: {
    margin: 10
  },
  overlay: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: '#00000088',
    justifyContent: 'center',
    alignItems: 'center',
    zIndex: 10
  }
});

export default App;
