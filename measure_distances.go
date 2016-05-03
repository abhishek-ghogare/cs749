package main

import (
    "io/ioutil"
    "fmt"
    "os"
    "strings"
    "strconv"
    "math"
)

const POINTXI = 2

func getCentroid(filename string) []float32 {

    file, err := ioutil.ReadFile(filename)
    if err != nil {
        fmt.Println("Input file does not exist : ", err.Error())
        os.Exit(1)
    }

    fileContent := string(file)
    fileContent = strings.Trim(fileContent, " \n\r")
    lines := strings.Split(fileContent, "\n")

    x :=float64(0)
    y :=float64(0)
    z :=float64(0)
    for _, line := range lines {
        if line=="" {
            continue
        }
        args := strings.Split(line, " ")
        var val float64
        if val, err = strconv.ParseFloat(args[POINTXI],64) ; err!=nil {
            fmt.Printf("Unable to parse %v:%v\n", args[POINTXI], err.Error())
        }
        x+=val
        if val, err = strconv.ParseFloat(args[POINTXI+1],64) ; err!=nil {
            fmt.Printf("Unable to parse %v:%v\n", args[POINTXI+1], err.Error())
        }
        y+=val
        if val, err = strconv.ParseFloat(args[POINTXI+2],64) ; err!=nil {
            fmt.Printf("Unable to parse %v:%v\n", args[POINTXI+2], err.Error())
        }
        z+=val
    }

    x /= float64(len(lines))
    y /= float64(len(lines))
    z /= float64(len(lines))
    fmt.Printf("avg : %v %v %v\n", x, y, z)
    return []float32{float32(x),float32(y),float32(z)}
}

type object struct {
    label int
    object int
    centroid []float32
}


func main() {
    objects := []object{}
    labels := 0

    files, err := ioutil.ReadDir("/tmp/objects/")
    if err != nil {
        fmt.Println(err.Error())
    }

    for _, file := range files {
        ob := object{}

        name := strings.FieldsFunc(file.Name(),func(c rune) bool {
            return c=='_' || c=='.' || c==' '
        })
        fmt.Println(name)
        ob.label, _ = strconv.Atoi(name[1])
        ob.object, _ = strconv.Atoi(name[3])

        ob.centroid = getCentroid("/tmp/objects/"+ file.Name())

        objects = append(objects, ob)
        if ob.label > labels {
            labels = ob.label
        }
    }
    /*
    lbl_obj := make([][]object, labels+1)

    for _, ob := range objects {
        lbl_obj[ob.label] = append(lbl_obj[ob.label], ob)
    }
    */
/*
    os.RemoveAll("/tmp/distances/")
    os.MkdirAll("/tmp/distances/",0777)*/

    filename := "/tmp/distances.data"
    os.Remove(filename)

    f, err := os.OpenFile(filename, os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0600)
    if err != nil {
        panic(err)
    }
    defer f.Close()
    writeToFile := func(line string){
        if _, err = f.WriteString(line); err != nil {
            panic(err)
        }
    }

    for _, ob1 := range objects {
        for  _, ob2 := range objects {
            if ob1.object == ob2.object {
                continue
            }
            distance := (ob1.centroid[0]- ob2.centroid[0]) * (ob1.centroid[0]- ob2.centroid[0])
            distance += (ob1.centroid[1]- ob2.centroid[1]) * (ob1.centroid[1]- ob2.centroid[1])
            distance += (ob1.centroid[2]- ob2.centroid[2]) * (ob1.centroid[2]- ob2.centroid[2])
            distance = float32(math.Sqrt(float64(distance)))
            str := fmt.Sprintf("%v %v %v %v %v\n", ob1.label, ob1.object, ob2.label, ob2.object, distance)
            writeToFile(str)
        }
    }
}
