/**
 * Instruction.cpp
 * Purpose: Information related to 'Instruction'
 * @author Mintae Kim
 */

#include "Instruction.h"

Instruction::Instruction()
{
	instType = InstructionType::ERR;
	paramCnt = 0;

	for (int i = 0; i < MAX_INST_PARAM; i++) param[i] = 0;
}
Instruction::Instruction(InstructionType instType, float* param)
{
	this->instType = instType;
	switch (instType)
	{
	// Server -> Client 
	case InstructionType::HLT:
		paramCnt = HLT_PARAM;
		break;
	case InstructionType::MOV:
		paramCnt = MOV_PARAM; // (cur_pos_x, cur_pos_y, cur_dir_x, cur_dir_y)
		break;
	case InstructionType::MVL:
		paramCnt = MVL_PARAM; // (cur_pos_x, cur_pos_y, cur_dir_x, cur_dir_y)
		break;
	case InstructionType::DCN:
		paramCnt = DCN_PARAM;
		break;

	// Server <-> Client
	case InstructionType::GRB:
		paramCnt = GRB_PARAM; // r, z
		break;
	case InstructionType::RLZ:
		paramCnt = RLZ_PARAM; // r. z
		break;
	case InstructionType::PID:
		paramCnt = PID_PARAM; // curPos, curDir, keypointPos, keypointDir
		break;


	// Tests
	case InstructionType::SET:
		paramCnt = SET_PARAM; // curPos, curDir, keypointPos, keypointDir
		break;
	}

	// Parameter Deep Copy
	for (int i = 0; i < paramCnt; i++) {
		(this->param)[i] = param[i];
	}
}
Instruction::Instruction(Instruction const& inst)
{
	paramCnt = inst.paramCnt;
	instType = inst.instType;

	// Parameter Deep Copy
	for (int i = 0; i < paramCnt; i++) {
		param[i] = inst.param[i];
	}
}
Instruction& Instruction::operator =(const Instruction& inst)
{
	paramCnt = inst.paramCnt;
	instType = inst.instType;

	// Parameter Deep Copy
	for (int i = 0; i < paramCnt; i++) {
		param[i] = inst.param[i];
	}

	return *this;
}

std::string Instruction::toString()
{
	std::string instRaw;
	switch (instType)
	{
	// Server -> Client 
	case InstructionType::HLT:
		instRaw = "HLT";
		break;
	case InstructionType::MOV:
		instRaw = "MOV";
		break;
	case InstructionType::MVL:
		instRaw = "MVL";
		break;
	case InstructionType::DCN:
		instRaw = "DCN";
		break;
	
	// Server <-> Client
	case InstructionType::GRB:
		instRaw = "GRB";
		break;
	case InstructionType::RLZ:
		instRaw = "RLZ";
		break;
	case InstructionType::PID:
		instRaw = "PID";
		break;

	// Tests
	case InstructionType::SET:
		instRaw = "SET";
		break;
	}

	for (int i = 0; i < paramCnt; i++) instRaw.append(" ").append(std::to_string(param[i]));
	return instRaw;
}