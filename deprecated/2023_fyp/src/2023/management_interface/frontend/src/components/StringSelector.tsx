import {useEffect, useState} from "react";
import {StringCard} from "./StringCard";

const StringSelector = ({items, setSelectedItems, multiselect=true}: StringSelectorProps) => {

    const [selected, setSelected] = useState<string[]>([])

    useEffect(() => setSelectedItems(selected), [selected])

    return (
        <div>
            {items.map(item => {

                const itemSelected = selected.includes(item)

                return (
                    <StringCard
                        selected={itemSelected}
                        text={item}
                        onClick={() => {
                            if (!multiselect) {
                                setSelected([item])
                            } else if (itemSelected) {
                                setSelected(selected.filter(val => item !== val))
                            } else {
                                setSelected(selected.concat(item))
                            }
                        }}
                    />
                )
            })}
        </div>

    )


}
export {StringSelector}

export type StringSelectorProps = {items: string[], setSelectedItems: (val: string[]) => void, multiselect?: boolean}