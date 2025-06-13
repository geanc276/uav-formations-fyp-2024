import React, {useState} from 'react';
import 'bootstrap/dist/css/bootstrap.min.css';
import {SoftwareTransfer} from "./pages/SoftwareTransfer";
import './App.css';
import {Tab, Tabs} from "react-bootstrap";
import {SoftwareSelector} from "./components/SoftwareSelector";
import {StartSoftware} from "./components/Start";
import {Drone} from "./components/DroneCard";


function App() {

    const [selectedDrones, setSelectedDrones] = useState<Drone[]>([])

    return (
        <div className="App">
            <header className="App-header">
                <h1>Management Interface</h1>
                <Tabs>
                    <Tab eventKey="software" title="Software" className="Tab">
                        <SoftwareSelector/>
                    </Tab>
                    <Tab eventKey="transfer" title="Transfer" className="Tab">
                        <SoftwareTransfer selectedDrones={selectedDrones} setSelectedDrones={setSelectedDrones}/>
                    </Tab>
                    <Tab eventKey="launch" title="Launch" className="Tab">
                        <StartSoftware selectedDrones={selectedDrones} setSelectedDrones={setSelectedDrones}/>
                    </Tab>

                </Tabs>
            </header>
        </div>
    );
}

export default App;
export const backendURL = "http://localhost:8000/"
