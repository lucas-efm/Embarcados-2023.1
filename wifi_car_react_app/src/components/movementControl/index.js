import React, { useState, useEffect } from 'react';
import firebase from 'firebase/compat/app';
import 'firebase/compat/database';

// Replace with your Firebase config
const firebaseConfig = {
  apiKey: "...",
  authDomain: "...",
  projectId: "...",
  storageBucket: "...",
  messagingSenderId: "...",
  appId: "...",
  measurementId: "..."
};


// Initialize Firebase
firebase.initializeApp(firebaseConfig);

const ButtonComponent = () => {
  const [movement, setMovement] = useState('');
  const [distance, setDistance] = useState('');

  useEffect(() => {
    const database = firebase.database();
    const movementRef = database.ref('movement');
    const distanceRef = database.ref('distance');

    // Listen for changes in the 'movement' value
    movementRef.on('value', (snapshot) => {
      const movementValue = snapshot.val();
      setMovement(movementValue || '');
    });

    // Listen for changes in the 'distance' value
    distanceRef.on('value', (snapshot) => {
      const distanceValue = snapshot.val();
      setDistance(distanceValue || '');
    });

    // Cleanup the listeners when component unmounts
    return () => {
      movementRef.off();
      distanceRef.off();
    };
  }, []);

  const handleButtonClick = (movementValue) => {
    const database = firebase.database();
    const movementRef = database.ref('movement');

    if (movementValue === 'STOP') {
      // Clear the movement value for stop
      movementRef.remove();
    } else if (movementValue === 'H') {
      // Toggle between 'H' and 'X' for the buzina button
      const newMovementValue = movement === 'H' ? 'X' : 'H';
      movementRef.set(newMovementValue);
    } else {
      // Update the movement value for other directions
      movementRef.set(movementValue);
    }
  };

  useEffect(() => {
    const handleKeyDown = (event) => {
      switch (event.code) {
        case 'KeyW':
          handleButtonClick('W');
          break;
        case 'KeyA':
          handleButtonClick('A');
          break;
        case 'KeyS':
          handleButtonClick('S');
          break;
        case 'KeyD':
          handleButtonClick('D');
          break;
        case 'Space':
          handleButtonClick('X');
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);

    // Cleanup the event listener when component unmounts
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  const buttonTitles = {
    W: 'Para frente',
    A: '90° Esquerda',
    X: 'Pare',
    D: '90° Direita',
    S: 'Para trás',
    H: 'Buzina',
  };

  return (
    <div className="container">
      <div className="button-group">
        <button className="button up" onClick={() => handleButtonClick('W')}>Para frente</button>
        <button className="button left" onClick={() => handleButtonClick('A')}>90° Esquerda</button>
        <button className="button stop" onClick={() => handleButtonClick('X')}>Pare</button>
        <button className="button right" onClick={() => handleButtonClick('D')}>90° Direita</button>
        <button className="button down" onClick={() => handleButtonClick('S')}>Para trás</button>
        <button className={`button buzina ${movement === 'H' ? 'active' : ''}`} onClick={() => handleButtonClick('H')}>Buzina</button>
      </div>

      <p className="movement-display">Último comando dado: {movement ? buttonTitles[movement] : 'Nenhum'}</p>

      <div className="info-group">
        <p className="info-label">Distancia do ultimo obstaculo:</p>
        <div className="distance-display">
          {distance &&
            distance.toString().split('').map((digit, index) => (
              <span className="digit" key={index}>
                {digit}
              </span>
            ))}
        </div>
        <p className="info-label">cm</p>
      </div>

      <style jsx>{`
        .container {
          display: flex;
          flex-direction: column;
          align-items: center;
          justify-content: center;
          height: 100vh;
          background-color: #333;
          position: relative;
          padding-top: 20px;
        }

        .button-group {
          display: grid;
          grid-template-columns: 1fr 1fr;
          grid-template-rows: 1fr 1fr;
          gap: 10px;
          margin-bottom: 20px;
        }

        .button {
          padding: 20px 20px;
          font-size: 16px;
          border: none;
          border-radius: 4px;
          background-color: #007bff;
          color: #fff;
          cursor: pointer;
        }

        .up {
          grid-row: 1;
          grid-column: 2;
        }

        .left {
          grid-row: 2;
          grid-column: 1;
        }

        .stop {
          grid-row: 2;
          grid-column: 2;
          background-color: #dc3545;
        }

        .right {
          grid-row: 2;
          grid-column: 3;
        }

        .down {
          grid-row: 3;
          grid-column: 2;
        }

        .buzina {
          position: absolute;
          top: 30%;
          right: 10%;
          width: 100px;
          height: 100px;
          border-radius: 50%;
          background-color: ${movement === 'H' ? 'orange' : 'yellow'};
          color: #333;
        }

        .movement-display {
          font-size: 18px;
          color: #fff;
        }

        .info-group {
          display: flex;
          align-items: center;
          margin-bottom: 10px;
        }

        .info-label {
          font-size: 18px;
          color: #fff;
          margin-right: 10px;
        }

        .distance-display {
          display: flex;
          align-items: center;
        }

        .digit {
          display: inline-block;
          width: 20px;
          height: 30px;
          margin-right: 2px;
          background-color: #222;
          color: #fff;
          font-size: 25px;
          text-align: center;
          line-height: 30px;
        }

        .active {
          background-color: orange;
        }
      `}</style>
    </div>
  );
};

export default ButtonComponent;