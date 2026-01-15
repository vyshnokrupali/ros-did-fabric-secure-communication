package main

import (
	"encoding/json"
	"fmt"

	"github.com/hyperledger/fabric-contract-api-go/contractapi"
)

/*
Device DID structure
*/
type Device struct {
	DID       string `json:"did"`
	PublicKey string `json:"publicKey"`
	Status    string `json:"status"`
}

/*
SmartContract definition
*/
type SmartContract struct {
	contractapi.Contract
}

/*
Register a new device DID
*/
func (s *SmartContract) RegisterDevice(
	ctx contractapi.TransactionContextInterface,
	did string,
	publicKey string,
) error {

	exists, err := s.DeviceExists(ctx, did)
	if err != nil {
		return err
	}
	if exists {
		return fmt.Errorf("device with DID %s already exists", did)
	}

	device := Device{
		DID:       did,
		PublicKey: publicKey,
		Status:    "VALID",
	}

	deviceJSON, err := json.Marshal(device)
	if err != nil {
		return err
	}

	return ctx.GetStub().PutState(did, deviceJSON)
}

/*
Verify whether a device is valid
*/
func (s *SmartContract) VerifyDevice(
	ctx contractapi.TransactionContextInterface,
	did string,
) (bool, error) {

	deviceJSON, err := ctx.GetStub().GetState(did)
	if err != nil {
		return false, err
	}
	if deviceJSON == nil {
		return false, nil
	}

	var device Device
	err = json.Unmarshal(deviceJSON, &device)
	if err != nil {
		return false, err
	}

	return device.Status == "VALID", nil
}

/*
Revoke a device DID
*/
func (s *SmartContract) RevokeDevice(
	ctx contractapi.TransactionContextInterface,
	did string,
) error {

	deviceJSON, err := ctx.GetStub().GetState(did)
	if err != nil {
		return err
	}
	if deviceJSON == nil {
		return fmt.Errorf("device with DID %s does not exist", did)
	}

	var device Device
	err = json.Unmarshal(deviceJSON, &device)
	if err != nil {
		return err
	}

	device.Status = "REVOKED"

	updatedJSON, err := json.Marshal(device)
	if err != nil {
		return err
	}

	return ctx.GetStub().PutState(did, updatedJSON)
}

/*
Check if a device exists
*/
func (s *SmartContract) DeviceExists(
	ctx contractapi.TransactionContextInterface,
	did string,
) (bool, error) {

	deviceJSON, err := ctx.GetStub().GetState(did)
	if err != nil {
		return false, err
	}

	return deviceJSON != nil, nil
}

/*
Chaincode entry point
*/
func main() {
	chaincode, err := contractapi.NewChaincode(new(SmartContract))
	if err != nil {
		panic(fmt.Sprintf("Error creating DID chaincode: %v", err))
	}

	if err := chaincode.Start(); err != nil {
		panic(fmt.Sprintf("Error starting DID chaincode: %v", err))
	}
}
