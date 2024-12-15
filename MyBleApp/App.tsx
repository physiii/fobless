import React, { useEffect, useState } from 'react';
import {
  View,
  Text,
  PermissionsAndroid,
  Platform,
  Alert,
  StyleSheet,
  Pressable
} from 'react-native';
import { BleManager, Device } from 'react-native-ble-plx';
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
        setConnectOn(true);
        startScan();
      } else {
        Alert.alert('Permissions required', 'Need all BLE and location permissions granted.');
      }
    }
    requestPermissions();

    return () => {
      manager.stopDeviceScan();
    };
  }, []);

  const startScan = () => {
    if (isScanning) {
      manager.stopDeviceScan();
      setIsScanning(false);
      return;
    }
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
        console.log('Found device:', device.name);
        manager.stopDeviceScan();
        setIsScanning(false);
        setScannedDevice(device);
        await connectToDevice(device);
      }
    });
  };

  const connectToDevice = async (device: Device) => {
    setIsConnecting(true);
    try {
      const d = await device.connect();
      console.log('Connected to:', d.name);
      await d.discoverAllServicesAndCharacteristics();
      setConnectedDevice(d);
      Alert.alert('Connected', `Successfully connected to ${d.name}`);
      await sendCommand(POWER_ON_CODE); // Power on automatically once connected
      setPowerOn(true);
    } catch (error) {
      console.error('Connection error:', error);
      Alert.alert('Connection Failed', 'Could not connect to the device.');
      setConnectOn(false);
    } finally {
      setIsConnecting(false);
    }
  };

  const disconnect = async () => {
    if (!connectedDevice) return;
    setIsConnecting(true);
    try {
      await connectedDevice.cancelConnection();
      setConnectedDevice(null);
      Alert.alert('Disconnected', 'The device has been disconnected.');
    } catch (err) {
      Alert.alert('Error', 'Failed to disconnect.');
      console.error(err);
      setConnectOn(true);
    } finally {
      setIsConnecting(false);
    }
  };

  const sendCommand = async (command: number) => {
    if (!connectedDevice) {
      Alert.alert('Not connected', 'Please connect first.');
      return;
    }
    if (!powerOn && command !== POWER_OFF_CODE && command !== POWER_ON_CODE) {
      Alert.alert('No Power', 'Power is off. Turn power on first.');
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
      Alert.alert('Error', 'Failed to write command.');
    } finally {
      setIsSending(false);
    }
  };

  const handleConnectToggle = (newVal: boolean) => {
    setConnectOn(newVal);
    if (newVal) {
      startScan();
    } else {
      disconnect();
    }
  };

  const handlePowerToggle = async (newVal: boolean) => {
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
            <MaterialCommunityIcons name={connectOn ? "bluetooth" : "bluetooth-off"} size={24} color="#fff" style={{marginRight:5}} />
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
              <MaterialCommunityIcons name="power" size={24} color="#fff" style={{marginRight:5}} />
              <Text style={styles.label}>Power On/Off</Text>
              <Switch 
                value={powerOn}
                onValueChange={handlePowerToggle}
                disabled={!canInteract}
              />
            </View>
          )}

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

          {!isConnected && !isScanning && !scannedDevice && (
            <Text style={styles.infoText}>
              Connecting to device...
            </Text>
          )}
          {scannedDevice && !isConnected && !isScanning && (
            <Text style={styles.infoText}>
              Found {scannedDevice.name}, connecting...
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
