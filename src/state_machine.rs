use std::sync::{Arc, Mutex};

#[derive(Debug)]
pub enum FSMError {
    InvalidTransition,
}

#[derive(Clone, Copy)]
pub enum SensorStatus {
    Bootup,
    PeripheralSetup,
    Sampling,
}

pub struct SensorFSM {
    state: SensorStatus,
}

impl SensorFSM {

    pub fn new() -> Self {
        Self {
            state: SensorStatus::Bootup,
        }
    }

    pub fn bootup_complete(&mut self) -> Result<(), FSMError> {
        match self.state {
            SensorStatus::Bootup => {
                self.state = SensorStatus::PeripheralSetup;
                Ok(())
            },
            _ => Err(FSMError::InvalidTransition)
        }
    }

    pub fn peripherals_complete(&mut self) -> Result<(), FSMError> {
        match self.state {
            SensorStatus::PeripheralSetup => {
                self.state = SensorStatus::Sampling;
                Ok(())
            },
            _ => Err(FSMError::InvalidTransition)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConnectionStatus {
    Bootup,
    PeripheralSetup,
    Connecting,
    Connected,
}

pub struct ConnectionFSM {
    state: Arc<Mutex<ConnectionStatus>>,
}

impl ConnectionFSM {

    pub fn new() -> Self {
        Self {
            state: Arc::new(Mutex::new(ConnectionStatus::Bootup)),
        }
    }

    pub fn get_observer(&self) -> Arc<Mutex<ConnectionStatus>> {
        self.state.clone()
    }

    pub fn bootup_complete(&mut self) -> Result<(), FSMError> {
        let mut st = self.state.lock().unwrap();
        match *st {
            ConnectionStatus::Bootup => {
                *st = ConnectionStatus::PeripheralSetup;
                Ok(())
            },
            _ => Err(FSMError::InvalidTransition)
        }
    }

    pub fn peripherals_complete(&mut self) -> Result<(), FSMError> {
        let mut st = self.state.lock().unwrap();
        match *st {
            ConnectionStatus::PeripheralSetup => {
                *st = ConnectionStatus::Connecting;
                Ok(())
            }
            _ => Err(FSMError::InvalidTransition)
        }
    }

    pub fn connected(&mut self) -> Result<(), FSMError> {
        let mut st = self.state.lock().unwrap();
        match *st {
            ConnectionStatus::Connecting => {
                *st = ConnectionStatus::Connected;
                Ok(())
            },
            _ => Err(FSMError::InvalidTransition),
        }
    }

    pub fn disconnected(&mut self) -> Result<(), FSMError> {
        let mut st = self.state.lock().unwrap();
        match *st {
            ConnectionStatus::Connected => {
                *st = ConnectionStatus::Connecting;
                Ok(())
            },
            _ => Err(FSMError::InvalidTransition),
        }
    }
}
