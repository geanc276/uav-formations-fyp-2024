import {DroneSelector, DroneSelectorProps} from "../components/DroneSelector";
import {useEffect, useState} from "react";
import {Button, Stack} from "react-bootstrap";
import {Drone} from "../components/DroneCard";
import {SoftwareSelector} from "../components/SoftwareSelector";
import {backendURL} from "../App";
import {StartSoftware} from "../components/Start";
import {StringSelector} from "../components/StringSelector";
import {encode} from "base-64";

const SoftwareTransfer = ({selectedDrones, setSelectedDrones}: DroneSelectorProps) => {

    const [images, setImages] = useState([])
    const [selectedImage, setSelectedImage] = useState<string[]>([])
    const [shouldDisableButton, setDisableButton] = useState(true)

    useEffect(() => getLocalImages(), [])
    useEffect(() => {
        setDisableButton(selectedImage.length === 0 || selectedDrones.length === 0)

    }, [selectedImage, selectedDrones])

    const getLocalImages = () => {
        fetch(backendURL + "getLocalImages", {
            headers: {
                Authorization: `Basic ${encode('test:testpassword')}`
            }
        })
            .then(result => {
                result.json().then(json => setImages(json))
            }).catch(e => console.log(e))
    }

    const beginTransfer = () => {

        fetch(backendURL + "startRegistry", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({
                image: selectedImage[0]
            })
        }).then(() => {

            const count = Array(selectedDrones.length)
            selectedDrones.forEach((value, index) => {
                fetch(backendURL + "pullSoftware", {
                    method: "POST",
                    headers: {"Content-Type": "application/json"},
                    body: JSON.stringify({
                        ip: value.ip,
                        image: selectedImage[0]
                    })
                }).then(() => {
                    count[index] = true
                    if (count.every(x => x)) {
                        // Enable button
                    }
                })
            })
        })


    }

    return (
        <div className="page">
            <DroneSelector selectedDrones={selectedDrones} setSelectedDrones={setSelectedDrones}/>
            <StringSelector items={images} setSelectedItems={setSelectedImage} multiselect={false}/>
            <Button className="mt-3" style={{width: "100%"}} disabled={shouldDisableButton} onClick={beginTransfer}>Transfer Software!</Button>
        </div>
    )
}

export {SoftwareTransfer}