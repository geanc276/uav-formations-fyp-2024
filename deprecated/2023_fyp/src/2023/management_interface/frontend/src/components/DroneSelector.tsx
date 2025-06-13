import {Button, Col, Container, Row} from "react-bootstrap";
import {Dispatch, SetStateAction, useEffect, useState} from "react";
import {Drone, DroneCard, DroneCardProps} from "./DroneCard";
import {Simulate} from "react-dom/test-utils";
import select = Simulate.select;
import {backendURL} from "../App";


const DroneSelector = ({selectedDrones, setSelectedDrones}:  DroneSelectorProps) => {

    const [drones, setDrones] = useState<{drone:Drone, selected:boolean}[]>([])


    useEffect(() => {
        const select = drones.filter(drone =>drone.selected).map(drone => drone.drone)
        setSelectedDrones(select)
    }, [drones, setSelectedDrones])

    useEffect(() => {
        getDrones()
    }, [])


    const getDrones = (force=false) => {

        fetch(backendURL + "getDrones" + (force ? "Force" :"")).then(response => {
            response.json().then(json => {
                setDrones(json.map((ip: string) => {
                    const drone = {
                        name: "Drone", ip, mac:""
                    };
                    return {drone, selected: false}

                }))
            })
        }).catch(e => console.log(e))
    }


    return (
        <div>
            <Button onClick={() => getDrones(true)}>Search for drones</Button>
            <div className="DroneSelector">
                <Container>
                    <Row>
                        {drones.map(item => {
                            return (
                                <Col xs={4}>
                                    <DroneCard onClick={() => {
                                        setDrones(drones.map((value) => {
                                            if (value === item) {
                                                return {drone: value.drone, selected: !value.selected}
                                            }
                                            return value
                                        }))
                                    }} selected={
                                        item.selected
                                    } drone={item.drone}/>
                                </Col>
                            )
                        })
                        }
                    </Row>
                </Container>


            </div>
        </div>
    )
}

    export {DroneSelector}
    export type DroneSelectorProps = {selectedDrones: Drone[], setSelectedDrones: (arg0: Drone[]) => void}