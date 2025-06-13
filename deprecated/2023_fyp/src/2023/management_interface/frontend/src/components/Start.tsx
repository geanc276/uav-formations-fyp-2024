import {Button, Form, Stack} from "react-bootstrap";
import {Drone} from "./DroneCard";
import {backendURL} from "../App";
import {useState} from "react";
import {DroneSelector, DroneSelectorProps} from "./DroneSelector";

const StartSoftware = ({selectedDrones, setSelectedDrones} : DroneSelectorProps) => {

    const [config, setConfig] = useState("05_angular_follow")

    const launchDrones = () => {

        selectedDrones.forEach((value, index) => {
            fetch(backendURL + "launchDrones", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify({ip: value.ip, number:index + 1, config})
            })
        })
    }

    return (
        <div>
            <DroneSelector selectedDrones={selectedDrones} setSelectedDrones={setSelectedDrones}/>
            <Stack direction="vertical">
                <Form.Label style={{textAlign: "left"}}>Config Folder</Form.Label>
                <Form.Control value={config} onChange={event => setConfig(event.target.value)}/>
            </Stack>

            <Button onClick={launchDrones}>Launch</Button>
        </div>
    )
}

export {StartSoftware}
export type StartSoftwareProps = {selectedDrones: Drone[]}