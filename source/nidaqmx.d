module nidaqmx;

private {/*import std}*/
	import std.stdio: stderr;
	import std.conv: to, text;
	import std.traits: ParameterTypeTuple, isSomeString;
	import std.typetuple: ReplaceAll;
	import std.string: toStringz;
	import std.range: empty;
	import std.algorithm: sort, findSplitBefore;
}
private {/*import evx}*/
	import evx.logic: not;
	import evx.utils: τ, function_call_to_string;
	import evx.meta: DynamicLibrary;
}
import evx.utils: writeln;

alias NIBool = uint;
alias TaskHandle = uint*;

struct DAQmx
	{/*...}*/
		__gshared nothrow:

		template opDispatch (string op, string file = __FILE__, uint line = __LINE__)
			{/*...}*/
				static opDispatch (Args...)(Args args)
					{/*...}*/
						alias CArgs = ReplaceAll!(string, const(char)*, Args);
						CArgs c_args;

						foreach (i, arg; args)
							{/*Args → CArgs}*/
								static if (isSomeString!(typeof(arg)))
									c_args[i] = args[i].toStringz;
								else c_args[i] = args[i];
							}

						enum generic_error = `call to DAQmxBase` ~op~ ` (` ~CArgs.stringof~ `) failed to compile`;
						static if (__traits(compiles, mixin(q{ParameterTypeTuple!(DAQmxBase} ~op~ q{)})))
							{/*enum error}*/
								mixin(q{
									alias Params = ParameterTypeTuple!(DAQmxBase} ~op~ q{);
								});

								static if (not(is(CArgs == Params)))
									enum error = `cannot call DAQmxBase` ~op~ ` ` ~Params.stringof~ ` with ` ~CArgs.stringof;
								else enum error = generic_error;
							}
						else enum error = generic_error;

						static assert (__traits(compiles, mixin(q{DAQmxBase} ~op~ q{ (c_args)})), error);

						version (LIVE) mixin(q{
							uint status = DAQmxBase} ~op~ q{ (c_args);
						}); else mixin (q{
							uint status = DAQmxSuccess;
						});

						if (status != DAQmxSuccess)
							{/*...}*/
								char[2^^8] error_string;

								DAQmxBaseGetExtendedErrorInfo (error_string.ptr, error_string.sizeof);

								string assert_msg;
								try assert_msg ~= `call to ` ~function_call_to_string!op (args)~ ` at ` ~file~ `:` ~line.text~ ` failed: ` ~error_string;
								catch (Exception) assert (0);

								assert (0, assert_msg);
							}
						else version (VERBOSE)
							try stderr.writeln (`called ` ~function_call_to_string!op (args));
							catch (Exception) assert (0);

						version (MOCK_DATA) {/*...}*/
							import evx.units;
							static size_t sample_counter;

							static if (op == `ReadAnalogF64`)
								{/*...}*/
									import std.math: isNaN;

									assert (not (mock_sampling_frequency.to_scalar.isNaN), `mock sampling frequency must be initialized with CfgSampClkTiming`);

									auto num_samps_per_chan = args[1];
									auto fill_mode = args[3];
									auto read_array = args[4];

									assert (fill_mode == DAQmx_Val_GroupByScanNumber);

									auto stride = mock_channel.length;

									foreach (i; 0..num_samps_per_chan)
										{/*write data}*/
											auto j = 0;

											import std.algorithm: map, zip;
											import std.range: array;

											try foreach (signal; zip (mock_channel.byKey, mock_channel.byValue).array
												.sort!((a,b) => a[0] < b[0])
												.map!(τ => τ[1])
											) read_array[i*stride + (j++)] = signal (i + num_samps_per_chan * sample_counter);
											catch (Exception) assert (0);
										}

									++sample_counter;

									sleep (num_samps_per_chan / mock_sampling_frequency);
								}
							else static if (op == `CreateAIVoltageChan`)
								{/*...}*/
									sample_counter = 0;

									auto channel_string = args[1];

									while (not (channel_string.empty))
										{/*...}*/
											auto split = channel_string[1..$].findSplitBefore (`,`);

											if (split[0].empty)
												break;

											uint n;
											try n = split[0][$-1..$].to!uint;
											catch (Exception) assert (0);

											if (not (n in mock_channel))
												{/*provide default}*/
													double delegate(uint) nothrow constant (uint x)
														{return i => x;}

													mock_channel[n] = constant (n);
												}

											channel_string = split[1];
										}
								}
							else static if (op == `CfgSampClkTiming`)
								{/*...}*/
									mock_sampling_frequency = args[2].hertz;
								}
						}
					}
			}

		version (MOCK_DATA) {/*...}*/
			import evx.units: Hertz;

			double delegate(size_t)[uint] mock_channel;
			Hertz mock_sampling_frequency;
		}

		private:
		enum {/*Error and Warning Codes}*/
			DAQmxSuccess = 0, 
			DAQmxErrorInvalidInstallation = -200683, 
			DAQmxErrorRefTrigMasterSessionUnavailable = -200682, 
			DAQmxErrorRouteFailedBecauseWatchdogExpired = -200681, 
			DAQmxErrorDeviceShutDownDueToHighTemp = -200680, 
			DAQmxErrorNoMemMapWhenHWTimedSinglePoint = -200679, 
			DAQmxErrorWriteFailedBecauseWatchdogExpired = -200678, 
			DAQmxErrorDifftInternalAIInputSrcs = -200677, 
			DAQmxErrorDifftAIInputSrcInOneChanGroup = -200676, 
			DAQmxErrorInternalAIInputSrcInMultChanGroups = -200675, 
			DAQmxErrorSwitchOpFailedDueToPrevError = -200674, 
			DAQmxErrorWroteMultiSampsUsingSingleSampWrite = -200673, 
			DAQmxErrorMismatchedInputArraySizes = -200672, 
			DAQmxErrorCantExceedRelayDriveLimit = -200671, 
			DAQmxErrorDACRngLowNotEqualToMinusRefVal = -200670, 
			DAQmxErrorCantAllowConnectDACToGnd = -200669, 
			DAQmxErrorWatchdogTimeoutOutOfRangeAndNotSpecialVal = -200668, 
			DAQmxErrorNoWatchdogOutputOnPortReservedForInput = -200667, 
			DAQmxErrorNoInputOnPortCfgdForWatchdogOutput = -200666, 
			DAQmxErrorWatchdogExpirationStateNotEqualForLinesInPort = -200665, 
			DAQmxErrorCannotPerformOpWhenTaskNotReserved = -200664, 
			DAQmxErrorPowerupStateNotSupported = -200663, 
			DAQmxErrorWatchdogTimerNotSupported = -200662, 
			DAQmxErrorOpNotSupportedWhenRefClkSrcNone = -200661, 
			DAQmxErrorSampClkRateUnavailable = -200660, 
			DAQmxErrorPrptyGetSpecdSingleActiveChanFailedDueToDifftVals = -200659, 
			DAQmxErrorPrptyGetImpliedActiveChanFailedDueToDifftVals = -200658, 
			DAQmxErrorPrptyGetSpecdActiveChanFailedDueToDifftVals = -200657, 
			DAQmxErrorNoRegenWhenUsingBrdMem = -200656, 
			DAQmxErrorNonbufferedReadMoreThanSampsPerChan = -200655, 
			DAQmxErrorWatchdogExpirationTristateNotSpecdForEntirePort = -200654, 
			DAQmxErrorPowerupTristateNotSpecdForEntirePort = -200653, 
			DAQmxErrorPowerupStateNotSpecdForEntirePort = -200652, 
			DAQmxErrorCantSetWatchdogExpirationOnDigInChan = -200651, 
			DAQmxErrorCantSetPowerupStateOnDigInChan = -200650, 
			DAQmxErrorPhysChanNotInTask = -200649, 
			DAQmxErrorPhysChanDevNotInTask = -200648, 
			DAQmxErrorDigInputNotSupported = -200647, 
			DAQmxErrorDigFilterIntervalNotEqualForLines = -200646, 
			DAQmxErrorDigFilterIntervalAlreadyCfgd = -200645, 
			DAQmxErrorCantResetExpiredWatchdog = -200644, 
			DAQmxErrorActiveChanTooManyLinesSpecdWhenGettingPrpty = -200643, 
			DAQmxErrorActiveChanNotSpecdWhenGetting1LinePrpty = -200642, 
			DAQmxErrorDigPrptyCannotBeSetPerLine = -200641, 
			DAQmxErrorSendAdvCmpltAfterWaitForTrigInScanlist = -200640, 
			DAQmxErrorDisconnectionRequiredInScanlist = -200639, 
			DAQmxErrorTwoWaitForTrigsAfterConnectionInScanlist = -200638, 
			DAQmxErrorActionSeparatorRequiredAfterBreakingConnectionInScanlist = -200637, 
			DAQmxErrorConnectionInScanlistMustWaitForTrig = -200636, 
			DAQmxErrorActionNotSupportedTaskNotWatchdog = -200635, 
			DAQmxErrorWfmNameSameAsScriptName = -200634, 
			DAQmxErrorScriptNameSameAsWfmName = -200633, 
			DAQmxErrorDSFStopClock = -200632, 
			DAQmxErrorDSFReadyForStartClock = -200631, 
			DAQmxErrorWriteOffsetNotMultOfIncr = -200630, 
			DAQmxErrorDifferentPrptyValsNotSupportedOnDev = -200629, 
			DAQmxErrorRefAndPauseTrigConfigured = -200628, 
			DAQmxErrorFailedToEnableHighSpeedInputClock = -200627, 
			DAQmxErrorEmptyPhysChanInPowerUpStatesArray = -200626, 
			DAQmxErrorActivePhysChanTooManyLinesSpecdWhenGettingPrpty = -200625, 
			DAQmxErrorActivePhysChanNotSpecdWhenGetting1LinePrpty = -200624, 
			DAQmxErrorPXIDevTempCausedShutDown = -200623, 
			DAQmxErrorInvalidNumSampsToWrite = -200622, 
			DAQmxErrorOutputFIFOUnderflow2 = -200621, 
			DAQmxErrorRepeatedAIPhysicalChan = -200620, 
			DAQmxErrorMultScanOpsInOneChassis = -200619, 
			DAQmxErrorInvalidAIChanOrder = -200618, 
			DAQmxErrorReversePowerProtectionActivated = -200617, 
			DAQmxErrorInvalidAsynOpHandle = -200616, 
			DAQmxErrorFailedToEnableHighSpeedOutput = -200615, 
			DAQmxErrorCannotReadPastEndOfRecord = -200614, 
			DAQmxErrorAcqStoppedToPreventInputBufferOverwriteOneDataXferMech = -200613, 
			DAQmxErrorZeroBasedChanIndexInvalid = -200612, 
			DAQmxErrorNoChansOfGivenTypeInTask = -200611, 
			DAQmxErrorSampClkSrcInvalidForOutputValidForInput = -200610, 
			DAQmxErrorOutputBufSizeTooSmallToStartGen = -200609, 
			DAQmxErrorInputBufSizeTooSmallToStartAcq = -200608, 
			DAQmxErrorExportTwoSignalsOnSameTerminal = -200607, 
			DAQmxErrorChanIndexInvalid = -200606, 
			DAQmxErrorRangeSyntaxNumberTooBig = -200605, 
			DAQmxErrorNULLPtr = -200604, 
			DAQmxErrorScaledMinEqualMax = -200603, 
			DAQmxErrorPreScaledMinEqualMax = -200602, 
			DAQmxErrorPropertyNotSupportedForScaleType = -200601, 
			DAQmxErrorChannelNameGenerationNumberTooBig = -200600, 
			DAQmxErrorRepeatedNumberInScaledValues = -200599, 
			DAQmxErrorRepeatedNumberInPreScaledValues = -200598, 
			DAQmxErrorLinesAlreadyReservedForOutput = -200597, 
			DAQmxErrorSwitchOperationChansSpanMultipleDevsInList = -200596, 
			DAQmxErrorInvalidIDInListAtBeginningOfSwitchOperation = -200595, 
			DAQmxErrorMStudioInvalidPolyDirection = -200594, 
			DAQmxErrorMStudioPropertyGetWhileTaskNotVerified = -200593, 
			DAQmxErrorRangeWithTooManyObjects = -200592, 
			DAQmxErrorCppDotNetAPINegativeBufferSize = -200591, 
			DAQmxErrorCppCantRemoveInvalidEventHandler = -200590, 
			DAQmxErrorCppCantRemoveEventHandlerTwice = -200589, 
			DAQmxErrorCppCantRemoveOtherObjectsEventHandler = -200588, 
			DAQmxErrorDigLinesReservedOrUnavailable = -200587, 
			DAQmxErrorDSFFailedToResetStream = -200586, 
			DAQmxErrorDSFReadyForOutputNotAsserted = -200585, 
			DAQmxErrorSampToWritePerChanNotMultipleOfIncr = -200584, 
			DAQmxErrorAOPropertiesCauseVoltageBelowMin = -200583, 
			DAQmxErrorAOPropertiesCauseVoltageOverMax = -200582, 
			DAQmxErrorPropertyNotSupportedWhenRefClkSrcNone = -200581, 
			DAQmxErrorAIMaxTooSmall = -200580, 
			DAQmxErrorAIMaxTooLarge = -200579, 
			DAQmxErrorAIMinTooSmall = -200578, 
			DAQmxErrorAIMinTooLarge = -200577, 
			DAQmxErrorBuiltInCJCSrcNotSupported = -200576, 
			DAQmxErrorTooManyPostTrigSampsPerChan = -200575, 
			DAQmxErrorTrigLineNotFoundSingleDevRoute = -200574, 
			DAQmxErrorDifferentInternalAIInputSources = -200573, 
			DAQmxErrorDifferentAIInputSrcInOneChanGroup = -200572, 
			DAQmxErrorInternalAIInputSrcInMultipleChanGroups = -200571, 
			DAQmxErrorCAPIChanIndexInvalid = -200570, 
			DAQmxErrorCollectionDoesNotMatchChanType = -200569, 
			DAQmxErrorOutputCantStartChangedRegenerationMode = -200568, 
			DAQmxErrorOutputCantStartChangedBufferSize = -200567, 
			DAQmxErrorChanSizeTooBigForU32PortWrite = -200566, 
			DAQmxErrorChanSizeTooBigForU8PortWrite = -200565, 
			DAQmxErrorChanSizeTooBigForU32PortRead = -200564, 
			DAQmxErrorChanSizeTooBigForU8PortRead = -200563, 
			DAQmxErrorInvalidDigDataWrite = -200562, 
			DAQmxErrorInvalidAODataWrite = -200561, 
			DAQmxErrorWaitUntilDoneDoesNotIndicateDone = -200560, 
			DAQmxErrorMultiChanTypesInTask = -200559, 
			DAQmxErrorMultiDevsInTask = -200558, 
			DAQmxErrorCannotSetPropertyWhenTaskRunning = -200557, 
			DAQmxErrorCannotGetPropertyWhenTaskNotCommittedOrRunning = -200556, 
			DAQmxErrorLeadingUnderscoreInString = -200555, 
			DAQmxErrorTrailingSpaceInString = -200554, 
			DAQmxErrorLeadingSpaceInString = -200553, 
			DAQmxErrorInvalidCharInString = -200552, 
			DAQmxErrorDLLBecameUnlocked = -200551, 
			DAQmxErrorDLLLock = -200550, 
			DAQmxErrorSelfCalConstsInvalid = -200549, 
			DAQmxErrorInvalidTrigCouplingExceptForExtTrigChan = -200548, 
			DAQmxErrorWriteFailsBufferSizeAutoConfigured = -200547, 
			DAQmxErrorExtCalAdjustExtRefVoltageFailed = -200546, 
			DAQmxErrorSelfCalFailedExtNoiseOrRefVoltageOutOfCal = -200545, 
			DAQmxErrorExtCalTemperatureNotDAQmx = -200544, 
			DAQmxErrorExtCalDateTimeNotDAQmx = -200543, 
			DAQmxErrorSelfCalTemperatureNotDAQmx = -200542, 
			DAQmxErrorSelfCalDateTimeNotDAQmx = -200541, 
			DAQmxErrorDACRefValNotSet = -200540, 
			DAQmxErrorAnalogMultiSampWriteNotSupported = -200539, 
			DAQmxErrorInvalidActionInControlTask = -200538, 
			DAQmxErrorPolyCoeffsInconsistent = -200537, 
			DAQmxErrorSensorValTooLow = -200536, 
			DAQmxErrorSensorValTooHigh = -200535, 
			DAQmxErrorWaveformNameTooLong = -200534, 
			DAQmxErrorIdentifierTooLongInScript = -200533, 
			DAQmxErrorUnexpectedIDFollowingSwitchChanName = -200532, 
			DAQmxErrorRelayNameNotSpecifiedInList = -200531, 
			DAQmxErrorUnexpectedIDFollowingRelayNameInList = -200530, 
			DAQmxErrorUnexpectedIDFollowingSwitchOpInList = -200529, 
			DAQmxErrorInvalidLineGrouping = -200528, 
			DAQmxErrorCtrMinMax = -200527, 
			DAQmxErrorWriteChanTypeMismatch = -200526, 
			DAQmxErrorReadChanTypeMismatch = -200525, 
			DAQmxErrorWriteNumChansMismatch = -200524, 
			DAQmxErrorOneChanReadForMultiChanTask = -200523, 
			DAQmxErrorCannotSelfCalDuringExtCal = -200522, 
			DAQmxErrorMeasCalAdjustOscillatorPhaseDAC = -200521, 
			DAQmxErrorInvalidCalConstCalADCAdjustment = -200520, 
			DAQmxErrorInvalidCalConstOscillatorFreqDACValue = -200519, 
			DAQmxErrorInvalidCalConstOscillatorPhaseDACValue = -200518, 
			DAQmxErrorInvalidCalConstOffsetDACValue = -200517, 
			DAQmxErrorInvalidCalConstGainDACValue = -200516, 
			DAQmxErrorInvalidNumCalADCReadsToAverage = -200515, 
			DAQmxErrorInvalidCfgCalAdjustDirectPathOutputImpedance = -200514, 
			DAQmxErrorInvalidCfgCalAdjustMainPathOutputImpedance = -200513, 
			DAQmxErrorInvalidCfgCalAdjustMainPathPostAmpGainAndOffset = -200512, 
			DAQmxErrorInvalidCfgCalAdjustMainPathPreAmpGain = -200511, 
			DAQmxErrorInvalidCfgCalAdjustMainPreAmpOffset = -200510, 
			DAQmxErrorMeasCalAdjustCalADC = -200509, 
			DAQmxErrorMeasCalAdjustOscillatorFrequency = -200508, 
			DAQmxErrorMeasCalAdjustDirectPathOutputImpedance = -200507, 
			DAQmxErrorMeasCalAdjustMainPathOutputImpedance = -200506, 
			DAQmxErrorMeasCalAdjustDirectPathGain = -200505, 
			DAQmxErrorMeasCalAdjustMainPathPostAmpGainAndOffset = -200504, 
			DAQmxErrorMeasCalAdjustMainPathPreAmpGain = -200503, 
			DAQmxErrorMeasCalAdjustMainPathPreAmpOffset = -200502, 
			DAQmxErrorInvalidDateTimeInEEPROM = -200501, 
			DAQmxErrorUnableToLocateErrorResources = -200500, 
			DAQmxErrorDotNetAPINotUnsigned32BitNumber = -200499, 
			DAQmxErrorInvalidRangeOfObjectsSyntaxInString = -200498, 
			DAQmxErrorAttemptToEnableLineNotPreviouslyDisabled = -200497, 
			DAQmxErrorInvalidCharInPattern = -200496, 
			DAQmxErrorIntermediateBufferFull = -200495, 
			DAQmxErrorLoadTaskFailsBecauseNoTimingOnDev = -200494, 
			DAQmxErrorCAPIReservedParamNotNULLNorEmpty = -200493, 
			DAQmxErrorCAPIReservedParamNotNULL = -200492, 
			DAQmxErrorCAPIReservedParamNotZero = -200491, 
			DAQmxErrorSampleValueOutOfRange = -200490, 
			DAQmxErrorChanAlreadyInTask = -200489, 
			DAQmxErrorVirtualChanDoesNotExist = -200488, 
			DAQmxErrorChanNotInTask = -200486, 
			DAQmxErrorTaskNotInDataNeighborhood = -200485, 
			DAQmxErrorCantSaveTaskWithoutReplace = -200484, 
			DAQmxErrorCantSaveChanWithoutReplace = -200483, 
			DAQmxErrorDevNotInTask = -200482, 
			DAQmxErrorDevAlreadyInTask = -200481, 
			DAQmxErrorCanNotPerformOpWhileTaskRunning = -200479, 
			DAQmxErrorCanNotPerformOpWhenNoChansInTask = -200478, 
			DAQmxErrorCanNotPerformOpWhenNoDevInTask = -200477, 
			DAQmxErrorCannotPerformOpWhenTaskNotRunning = -200475, 
			DAQmxErrorOperationTimedOut = -200474, 
			DAQmxErrorCannotReadWhenAutoStartFalseAndTaskNotRunningOrCommitted = -200473, 
			DAQmxErrorCannotWriteWhenAutoStartFalseAndTaskNotRunningOrCommitted = -200472, 
			DAQmxErrorTaskVersionNew = -200470, 
			DAQmxErrorChanVersionNew = -200469, 
			DAQmxErrorEmptyString = -200467, 
			DAQmxErrorChannelSizeTooBigForPortReadType = -200466, 
			DAQmxErrorChannelSizeTooBigForPortWriteType = -200465, 
			DAQmxErrorExpectedNumberOfChannelsVerificationFailed = -200464, 
			DAQmxErrorNumLinesMismatchInReadOrWrite = -200463, 
			DAQmxErrorOutputBufferEmpty = -200462, 
			DAQmxErrorInvalidChanName = -200461, 
			DAQmxErrorReadNoInputChansInTask = -200460, 
			DAQmxErrorWriteNoOutputChansInTask = -200459, 
			DAQmxErrorPropertyNotSupportedNotInputTask = -200457, 
			DAQmxErrorPropertyNotSupportedNotOutputTask = -200456, 
			DAQmxErrorGetPropertyNotInputBufferedTask = -200455, 
			DAQmxErrorGetPropertyNotOutputBufferedTask = -200454, 
			DAQmxErrorInvalidTimeoutVal = -200453, 
			DAQmxErrorAttributeNotSupportedInTaskContext = -200452, 
			DAQmxErrorAttributeNotQueryableUnlessTaskIsCommitted = -200451, 
			DAQmxErrorAttributeNotSettableWhenTaskIsRunning = -200450, 
			DAQmxErrorDACRngLowNotMinusRefValNorZero = -200449, 
			DAQmxErrorDACRngHighNotEqualRefVal = -200448, 
			DAQmxErrorUnitsNotFromCustomScale = -200447, 
			DAQmxErrorInvalidVoltageReadingDuringExtCal = -200446, 
			DAQmxErrorCalFunctionNotSupported = -200445, 
			DAQmxErrorInvalidPhysicalChanForCal = -200444, 
			DAQmxErrorExtCalNotComplete = -200443, 
			DAQmxErrorCantSyncToExtStimulusFreqDuringCal = -200442, 
			DAQmxErrorUnableToDetectExtStimulusFreqDuringCal = -200441, 
			DAQmxErrorInvalidCloseAction = -200440, 
			DAQmxErrorExtCalFunctionOutsideExtCalSession = -200439, 
			DAQmxErrorInvalidCalArea = -200438, 
			DAQmxErrorExtCalConstsInvalid = -200437, 
			DAQmxErrorStartTrigDelayWithExtSampClk = -200436, 
			DAQmxErrorDelayFromSampClkWithExtConv = -200435, 
			DAQmxErrorFewerThan2PreScaledVals = -200434, 
			DAQmxErrorFewerThan2ScaledValues = -200433, 
			DAQmxErrorPhysChanOutputType = -200432, 
			DAQmxErrorPhysChanMeasType = -200431, 
			DAQmxErrorInvalidPhysChanType = -200430, 
			DAQmxErrorLabVIEWEmptyTaskOrChans = -200429, 
			DAQmxErrorLabVIEWInvalidTaskOrChans = -200428, 
			DAQmxErrorInvalidRefClkRate = -200427, 
			DAQmxErrorInvalidExtTrigImpedance = -200426, 
			DAQmxErrorHystTrigLevelAIMax = -200425, 
			DAQmxErrorLineNumIncompatibleWithVideoSignalFormat = -200424, 
			DAQmxErrorTrigWindowAIMinAIMaxCombo = -200423, 
			DAQmxErrorTrigAIMinAIMax = -200422, 
			DAQmxErrorHystTrigLevelAIMin = -200421, 
			DAQmxErrorInvalidSampRateConsiderRIS = -200420, 
			DAQmxErrorInvalidReadPosDuringRIS = -200419, 
			DAQmxErrorImmedTrigDuringRISMode = -200418, 
			DAQmxErrorTDCNotEnabledDuringRISMode = -200417, 
			DAQmxErrorMultiRecWithRIS = -200416, 
			DAQmxErrorInvalidRefClkSrc = -200415, 
			DAQmxErrorInvalidSampClkSrc = -200414, 
			DAQmxErrorInsufficientOnBoardMemForNumRecsAndSamps = -200413, 
			DAQmxErrorInvalidAIAttenuation = -200412, 
			DAQmxErrorACCouplingNotAllowedWith50OhmImpedance = -200411, 
			DAQmxErrorInvalidRecordNum = -200410, 
			DAQmxErrorZeroSlopeLinearScale = -200409, 
			DAQmxErrorZeroReversePolyScaleCoeffs = -200408, 
			DAQmxErrorZeroForwardPolyScaleCoeffs = -200407, 
			DAQmxErrorNoReversePolyScaleCoeffs = -200406, 
			DAQmxErrorNoForwardPolyScaleCoeffs = -200405, 
			DAQmxErrorNoPolyScaleCoeffs = -200404, 
			DAQmxErrorReversePolyOrderLessThanNumPtsToCompute = -200403, 
			DAQmxErrorReversePolyOrderNotPositive = -200402, 
			DAQmxErrorNumPtsToComputeNotPositive = -200401, 
			DAQmxErrorWaveformLengthNotMultipleOfIncr = -200400, 
			DAQmxErrorCAPINoExtendedErrorInfoAvailable = -200399, 
			DAQmxErrorCVIFunctionNotFoundInDAQmxDLL = -200398, 
			DAQmxErrorCVIFailedToLoadDAQmxDLL = -200397, 
			DAQmxErrorNoCommonTrigLineForImmedRoute = -200396, 
			DAQmxErrorNoCommonTrigLineForTaskRoute = -200395, 
			DAQmxErrorF64PrptyValNotUnsignedInt = -200394, 
			DAQmxErrorRegisterNotWritable = -200393, 
			DAQmxErrorInvalidOutputVoltageAtSampClkRate = -200392, 
			DAQmxErrorStrobePhaseShiftDCMBecameUnlocked = -200391, 
			DAQmxErrorDrivePhaseShiftDCMBecameUnlocked = -200390, 
			DAQmxErrorClkOutPhaseShiftDCMBecameUnlocked = -200389, 
			DAQmxErrorOutputBoardClkDCMBecameUnlocked = -200388, 
			DAQmxErrorInputBoardClkDCMBecameUnlocked = -200387, 
			DAQmxErrorInternalClkDCMBecameUnlocked = -200386, 
			DAQmxErrorDCMLock = -200385, 
			DAQmxErrorDataLineReservedForDynamicOutput = -200384, 
			DAQmxErrorInvalidRefClkSrcGivenSampClkSrc = -200383, 
			DAQmxErrorNoPatternMatcherAvailable = -200382, 
			DAQmxErrorInvalidDelaySampRateBelowPhaseShiftDCMThresh = -200381, 
			DAQmxErrorStrainGageCalibration = -200380, 
			DAQmxErrorInvalidExtClockFreqAndDivCombo = -200379, 
			DAQmxErrorCustomScaleDoesNotExist = -200378, 
			DAQmxErrorOnlyFrontEndChanOpsDuringScan = -200377, 
			DAQmxErrorInvalidOptionForDigitalPortChannel = -200376, 
			DAQmxErrorUnsupportedSignalTypeExportSignal = -200375, 
			DAQmxErrorInvalidSignalTypeExportSignal = -200374, 
			DAQmxErrorUnsupportedTrigTypeSendsSWTrig = -200373, 
			DAQmxErrorInvalidTrigTypeSendsSWTrig = -200372, 
			DAQmxErrorRepeatedPhysicalChan = -200371, 
			DAQmxErrorResourcesInUseForRouteInTask = -200370, 
			DAQmxErrorResourcesInUseForRoute = -200369, 
			DAQmxErrorRouteNotSupportedByHW = -200368, 
			DAQmxErrorResourcesInUseForExportSignalPolarity = -200367, 
			DAQmxErrorResourcesInUseForInversionInTask = -200366, 
			DAQmxErrorResourcesInUseForInversion = -200365, 
			DAQmxErrorExportSignalPolarityNotSupportedByHW = -200364, 
			DAQmxErrorInversionNotSupportedByHW = -200363, 
			DAQmxErrorOverloadedChansExistNotRead = -200362, 
			DAQmxErrorInputFIFOOverflow2 = -200361, 
			DAQmxErrorCJCChanNotSpecd = -200360, 
			DAQmxErrorCtrExportSignalNotPossible = -200359, 
			DAQmxErrorRefTrigWhenContinuous = -200358, 
			DAQmxErrorIncompatibleSensorOutputAndDeviceInputRanges = -200357, 
			DAQmxErrorCustomScaleNameUsed = -200356, 
			DAQmxErrorPropertyValNotSupportedByHW = -200355, 
			DAQmxErrorPropertyValNotValidTermName = -200354, 
			DAQmxErrorResourcesInUseForProperty = -200353, 
			DAQmxErrorCJCChanAlreadyUsed = -200352, 
			DAQmxErrorForwardPolynomialCoefNotSpecd = -200351, 
			DAQmxErrorTableScaleNumPreScaledAndScaledValsNotEqual = -200350, 
			DAQmxErrorTableScalePreScaledValsNotSpecd = -200349, 
			DAQmxErrorTableScaleScaledValsNotSpecd = -200348, 
			DAQmxErrorIntermediateBufferSizeNotMultipleOfIncr = -200347, 
			DAQmxErrorEventPulseWidthOutOfRange = -200346, 
			DAQmxErrorEventDelayOutOfRange = -200345, 
			DAQmxErrorSampPerChanNotMultipleOfIncr = -200344, 
			DAQmxErrorCannotCalculateNumSampsTaskNotStarted = -200343, 
			DAQmxErrorScriptNotInMem = -200342, 
			DAQmxErrorOnboardMemTooSmall = -200341, 
			DAQmxErrorReadAllAvailableDataWithoutBuffer = -200340, 
			DAQmxErrorPulseActiveAtStart = -200339, 
			DAQmxErrorCalTempNotSupported = -200338, 
			DAQmxErrorDelayFromSampClkTooLong = -200337, 
			DAQmxErrorDelayFromSampClkTooShort = -200336, 
			DAQmxErrorAIConvRateTooHigh = -200335, 
			DAQmxErrorDelayFromStartTrigTooLong = -200334, 
			DAQmxErrorDelayFromStartTrigTooShort = -200333, 
			DAQmxErrorSampRateTooHigh = -200332, 
			DAQmxErrorSampRateTooLow = -200331, 
			DAQmxErrorPFI0UsedForAnalogAndDigitalSrc = -200330, 
			DAQmxErrorPrimingCfgFIFO = -200329, 
			DAQmxErrorCannotOpenTopologyCfgFile = -200328, 
			DAQmxErrorInvalidDTInsideWfmDataType = -200327, 
			DAQmxErrorRouteSrcAndDestSame = -200326, 
			DAQmxErrorReversePolynomialCoefNotSpecd = -200325, 
			DAQmxErrorDevAbsentOrUnavailable = -200324, 
			DAQmxErrorNoAdvTrigForMultiDevScan = -200323, 
			DAQmxErrorInterruptsInsufficientDataXferMech = -200322, 
			DAQmxErrorInvalidAttentuationBasedOnMinMax = -200321, 
			DAQmxErrorCabledModuleCannotRouteSSH = -200320, 
			DAQmxErrorCabledModuleCannotRouteConvClk = -200319, 
			DAQmxErrorInvalidExcitValForScaling = -200318, 
			DAQmxErrorNoDevMemForScript = -200317, 
			DAQmxErrorScriptDataUnderflow = -200316, 
			DAQmxErrorNoDevMemForWaveform = -200315, 
			DAQmxErrorStreamDCMBecameUnlocked = -200314, 
			DAQmxErrorStreamDCMLock = -200313, 
			DAQmxErrorWaveformNotInMem = -200312, 
			DAQmxErrorWaveformWriteOutOfBounds = -200311, 
			DAQmxErrorWaveformPreviouslyAllocated = -200310, 
			DAQmxErrorSampClkTbMasterTbDivNotAppropriateForSampTbSrc = -200309, 
			DAQmxErrorSampTbRateSampTbSrcMismatch = -200308, 
			DAQmxErrorMasterTbRateMasterTbSrcMismatch = -200307, 
			DAQmxErrorSampsPerChanTooBig = -200306, 
			DAQmxErrorFinitePulseTrainNotPossible = -200305, 
			DAQmxErrorExtMasterTimebaseRateNotSpecified = -200304, 
			DAQmxErrorExtSampClkSrcNotSpecified = -200303, 
			DAQmxErrorInputSignalSlowerThanMeasTime = -200302, 
			DAQmxErrorCannotUpdatePulseGenProperty = -200301, 
			DAQmxErrorInvalidTimingType = -200300, 
			DAQmxErrorPropertyUnavailWhenUsingOnboardMemory = -200297, 
			DAQmxErrorCannotWriteAfterStartWithOnboardMemory = -200295, 
			DAQmxErrorNotEnoughSampsWrittenForInitialXferRqstCondition = -200294, 
			DAQmxErrorNoMoreSpace = -200293, 
			DAQmxErrorSamplesCanNotYetBeWritten = -200292, 
			DAQmxErrorGenStoppedToPreventIntermediateBufferRegenOfOldSamples = -200291, 
			DAQmxErrorGenStoppedToPreventRegenOfOldSamples = -200290, 
			DAQmxErrorSamplesNoLongerWriteable = -200289, 
			DAQmxErrorSamplesWillNeverBeGenerated = -200288, 
			DAQmxErrorNegativeWriteSampleNumber = -200287, 
			DAQmxErrorNoAcqStarted = -200286, 
			DAQmxErrorSamplesNotYetAvailable = -200284, 
			DAQmxErrorAcqStoppedToPreventIntermediateBufferOverflow = -200283, 
			DAQmxErrorNoRefTrigConfigured = -200282, 
			DAQmxErrorCannotReadRelativeToRefTrigUntilDone = -200281, 
			DAQmxErrorSamplesNoLongerAvailable = -200279, 
			DAQmxErrorSamplesWillNeverBeAvailable = -200278, 
			DAQmxErrorNegativeReadSampleNumber = -200277, 
			DAQmxErrorExternalSampClkAndRefClkThruSameTerm = -200276, 
			DAQmxErrorExtSampClkRateTooLowForClkIn = -200275, 
			DAQmxErrorExtSampClkRateTooHighForBackplane = -200274, 
			DAQmxErrorSampClkRateAndDivCombo = -200273, 
			DAQmxErrorSampClkRateTooLowForDivDown = -200272, 
			DAQmxErrorProductOfAOMinAndGainTooSmall = -200271, 
			DAQmxErrorInterpolationRateNotPossible = -200270, 
			DAQmxErrorOffsetTooLarge = -200269, 
			DAQmxErrorOffsetTooSmall = -200268, 
			DAQmxErrorProductOfAOMaxAndGainTooLarge = -200267, 
			DAQmxErrorMinAndMaxNotSymmetric = -200266, 
			DAQmxErrorInvalidAnalogTrigSrc = -200265, 
			DAQmxErrorTooManyChansForAnalogRefTrig = -200264, 
			DAQmxErrorTooManyChansForAnalogPauseTrig = -200263, 
			DAQmxErrorTrigWhenOnDemandSampTiming = -200262, 
			DAQmxErrorInconsistentAnalogTrigSettings = -200261, 
			DAQmxErrorMemMapDataXferModeSampTimingCombo = -200260, 
			DAQmxErrorInvalidJumperedAttr = -200259, 
			DAQmxErrorInvalidGainBasedOnMinMax = -200258, 
			DAQmxErrorInconsistentExcit = -200257, 
			DAQmxErrorTopologyNotSupportedByCfgTermBlock = -200256, 
			DAQmxErrorBuiltInTempSensorNotSupported = -200255, 
			DAQmxErrorInvalidTerm = -200254, 
			DAQmxErrorCannotTristateTerm = -200253, 
			DAQmxErrorCannotTristateBusyTerm = -200252, 
			DAQmxErrorNoDMAChansAvailable = -200251, 
			DAQmxErrorInvalidWaveformLengthWithinLoopInScript = -200250, 
			DAQmxErrorInvalidSubsetLengthWithinLoopInScript = -200249, 
			DAQmxErrorMarkerPosInvalidForLoopInScript = -200248, 
			DAQmxErrorIntegerExpectedInScript = -200247, 
			DAQmxErrorPLLBecameUnlocked = -200246, 
			DAQmxErrorPLLLock = -200245, 
			DAQmxErrorDDCClkOutDCMBecameUnlocked = -200244, 
			DAQmxErrorDDCClkOutDCMLock = -200243, 
			DAQmxErrorClkDoublerDCMBecameUnlocked = -200242, 
			DAQmxErrorClkDoublerDCMLock = -200241, 
			DAQmxErrorSampClkDCMBecameUnlocked = -200240, 
			DAQmxErrorSampClkDCMLock = -200239, 
			DAQmxErrorSampClkTimebaseDCMBecameUnlocked = -200238, 
			DAQmxErrorSampClkTimebaseDCMLock = -200237, 
			DAQmxErrorAttrCannotBeReset = -200236, 
			DAQmxErrorExplanationNotFound = -200235, 
			DAQmxErrorWriteBufferTooSmall = -200234, 
			DAQmxErrorSpecifiedAttrNotValid = -200233, 
			DAQmxErrorAttrCannotBeRead = -200232, 
			DAQmxErrorAttrCannotBeSet = -200231, 
			DAQmxErrorNULLPtrForC_Api = -200230, 
			DAQmxErrorReadBufferTooSmall = -200229, 
			DAQmxErrorBufferTooSmallForString = -200228, 
			DAQmxErrorNoAvailTrigLinesOnDevice = -200227, 
			DAQmxErrorTrigBusLineNotAvail = -200226, 
			DAQmxErrorCouldNotReserveRequestedTrigLine = -200225, 
			DAQmxErrorTrigLineNotFound = -200224, 
			DAQmxErrorSCXI1126ThreshHystCombination = -200223, 
			DAQmxErrorAcqStoppedToPreventInputBufferOverwrite = -200222, 
			DAQmxErrorTimeoutExceeded = -200221, 
			DAQmxErrorInvalidDeviceID = -200220, 
			DAQmxErrorInvalidAOChanOrder = -200219, 
			DAQmxErrorSampleTimingTypeAndDataXferMode = -200218, 
			DAQmxErrorBufferWithOnDemandSampTiming = -200217, 
			DAQmxErrorBufferAndDataXferMode = -200216, 
			DAQmxErrorMemMapAndBuffer = -200215, 
			DAQmxErrorNoAnalogTrigHW = -200214, 
			DAQmxErrorTooManyPretrigPlusMinPostTrigSamps = -200213, 
			DAQmxErrorInconsistentUnitsSpecified = -200212, 
			DAQmxErrorMultipleRelaysForSingleRelayOp = -200211, 
			DAQmxErrorMultipleDevIDsPerChassisSpecifiedInList = -200210, 
			DAQmxErrorDuplicateDevIDInList = -200209, 
			DAQmxErrorInvalidRangeStatementCharInList = -200208, 
			DAQmxErrorInvalidDeviceIDInList = -200207, 
			DAQmxErrorTriggerPolarityConflict = -200206, 
			DAQmxErrorCannotScanWithCurrentTopology = -200205, 
			DAQmxErrorUnexpectedIdentifierInFullySpecifiedPathInList = -200204, 
			DAQmxErrorSwitchCannotDriveMultipleTrigLines = -200203, 
			DAQmxErrorInvalidRelayName = -200202, 
			DAQmxErrorSwitchScanlistTooBig = -200201, 
			DAQmxErrorSwitchChanInUse = -200200, 
			DAQmxErrorSwitchNotResetBeforeScan = -200199, 
			DAQmxErrorInvalidTopology = -200198, 
			DAQmxErrorAttrNotSupported = -200197, 
			DAQmxErrorUnexpectedEndOfActionsInList = -200196, 
			DAQmxErrorPowerBudgetExceeded = -200195, 
			DAQmxErrorHWUnexpectedlyPoweredOffAndOn = -200194, 
			DAQmxErrorSwitchOperationNotSupported = -200193, 
			DAQmxErrorOnlyContinuousScanSupported = -200192, 
			DAQmxErrorSwitchDifferentTopologyWhenScanning = -200191, 
			DAQmxErrorDisconnectPathNotSameAsExistingPath = -200190, 
			DAQmxErrorConnectionNotPermittedOnChanReservedForRouting = -200189, 
			DAQmxErrorCannotConnectSrcChans = -200188, 
			DAQmxErrorCannotConnectChannelToItself = -200187, 
			DAQmxErrorChannelNotReservedForRouting = -200186, 
			DAQmxErrorCannotConnectChansDirectly = -200185, 
			DAQmxErrorChansAlreadyConnected = -200184, 
			DAQmxErrorChanDuplicatedInPath = -200183, 
			DAQmxErrorNoPathToDisconnect = -200182, 
			DAQmxErrorInvalidSwitchChan = -200181, 
			DAQmxErrorNoPathAvailableBetween2SwitchChans = -200180, 
			DAQmxErrorExplicitConnectionExists = -200179, 
			DAQmxErrorSwitchDifferentSettlingTimeWhenScanning = -200178, 
			DAQmxErrorOperationOnlyPermittedWhileScanning = -200177, 
			DAQmxErrorOperationNotPermittedWhileScanning = -200176, 
			DAQmxErrorHardwareNotResponding = -200175, 
			DAQmxErrorInvalidSampAndMasterTimebaseRateCombo = -200173, 
			DAQmxErrorNonZeroBufferSizeInProgIOXfer = -200172, 
			DAQmxErrorVirtualChanNameUsed = -200171, 
			DAQmxErrorPhysicalChanDoesNotExist = -200170, 
			DAQmxErrorMemMapOnlyForProgIOXfer = -200169, 
			DAQmxErrorTooManyChans = -200168, 
			DAQmxErrorCannotHaveCJTempWithOtherChans = -200167, 
			DAQmxErrorOutputBufferUnderwrite = -200166, 
			DAQmxErrorSensorInvalidCompletionResistance = -200163, 
			DAQmxErrorVoltageExcitIncompatibleWith2WireCfg = -200162, 
			DAQmxErrorIntExcitSrcNotAvailable = -200161, 
			DAQmxErrorCannotCreateChannelAfterTaskVerified = -200160, 
			DAQmxErrorLinesReservedForSCXIControl = -200159, 
			DAQmxErrorCouldNotReserveLinesForSCXIControl = -200158, 
			DAQmxErrorCalibrationFailed = -200157, 
			DAQmxErrorReferenceFrequencyInvalid = -200156, 
			DAQmxErrorReferenceResistanceInvalid = -200155, 
			DAQmxErrorReferenceCurrentInvalid = -200154, 
			DAQmxErrorReferenceVoltageInvalid = -200153, 
			DAQmxErrorEEPROMDataInvalid = -200152, 
			DAQmxErrorCabledModuleNotCapableOfRoutingAI = -200151, 
			DAQmxErrorChannelNotAvailableInParallelMode = -200150, 
			DAQmxErrorExternalTimebaseRateNotKnownForDelay = -200149, 
			DAQmxErrorFREQOUTCannotProduceDesiredFrequency = -200148, 
			DAQmxErrorMultipleCounterInputTask = -200147, 
			DAQmxErrorCounterStartPauseTriggerConflict = -200146, 
			DAQmxErrorCounterInputPauseTriggerAndSampleClockInvalid = -200145, 
			DAQmxErrorCounterOutputPauseTriggerInvalid = -200144, 
			DAQmxErrorCounterTimebaseRateNotSpecified = -200143, 
			DAQmxErrorCounterTimebaseRateNotFound = -200142, 
			DAQmxErrorCounterOverflow = -200141, 
			DAQmxErrorCounterNoTimebaseEdgesBetweenGates = -200140, 
			DAQmxErrorCounterMaxMinRangeFreq = -200139, 
			DAQmxErrorCounterMaxMinRangeTime = -200138, 
			DAQmxErrorSuitableTimebaseNotFoundTimeCombo = -200137, 
			DAQmxErrorSuitableTimebaseNotFoundFrequencyCombo = -200136, 
			DAQmxErrorInternalTimebaseSourceDivisorCombo = -200135, 
			DAQmxErrorInternalTimebaseSourceRateCombo = -200134, 
			DAQmxErrorInternalTimebaseRateDivisorSourceCombo = -200133, 
			DAQmxErrorExternalTimebaseRateNotknownForRate = -200132, 
			DAQmxErrorAnalogTrigChanNotFirstInScanList = -200131, 
			DAQmxErrorNoDivisorForExternalSignal = -200130, 
			DAQmxErrorAttributeInconsistentAcrossRepeatedPhysicalChannels = -200128, 
			DAQmxErrorCannotHandshakeWithPort0 = -200127, 
			DAQmxErrorControlLineConflictOnPortC = -200126, 
			DAQmxErrorLines4To7ConfiguredForOutput = -200125, 
			DAQmxErrorLines4To7ConfiguredForInput = -200124, 
			DAQmxErrorLines0To3ConfiguredForOutput = -200123, 
			DAQmxErrorLines0To3ConfiguredForInput = -200122, 
			DAQmxErrorPortConfiguredForOutput = -200121, 
			DAQmxErrorPortConfiguredForInput = -200120, 
			DAQmxErrorPortConfiguredForStaticDigitalOps = -200119, 
			DAQmxErrorPortReservedForHandshaking = -200118, 
			DAQmxErrorPortDoesNotSupportHandshakingDataIO = -200117, 
			DAQmxErrorCannotTristate8255OutputLines = -200116, 
			DAQmxErrorTemperatureOutOfRangeForCalibration = -200113, 
			DAQmxErrorCalibrationHandleInvalid = -200112, 
			DAQmxErrorPasswordRequired = -200111, 
			DAQmxErrorIncorrectPassword = -200110, 
			DAQmxErrorPasswordTooLong = -200109, 
			DAQmxErrorCalibrationSessionAlreadyOpen = -200108, 
			DAQmxErrorSCXIModuleIncorrect = -200107, 
			DAQmxErrorAttributeInconsistentAcrossChannelsOnDevice = -200106, 
			DAQmxErrorSCXI1122ResistanceChanNotSupportedForCfg = -200105, 
			DAQmxErrorBracketPairingMismatchInList = -200104, 
			DAQmxErrorInconsistentNumSamplesToWrite = -200103, 
			DAQmxErrorIncorrectDigitalPattern = -200102, 
			DAQmxErrorIncorrectNumChannelsToWrite = -200101, 
			DAQmxErrorIncorrectReadFunction = -200100, 
			DAQmxErrorPhysicalChannelNotSpecified = -200099, 
			DAQmxErrorMoreThanOneTerminal = -200098, 
			DAQmxErrorMoreThanOneActiveChannelSpecified = -200097, 
			DAQmxErrorInvalidNumberSamplesToRead = -200096, 
			DAQmxErrorAnalogWaveformExpected = -200095, 
			DAQmxErrorDigitalWaveformExpected = -200094, 
			DAQmxErrorActiveChannelNotSpecified = -200093, 
			DAQmxErrorFunctionNotSupportedForDeviceTasks = -200092, 
			DAQmxErrorFunctionNotInLibrary = -200091, 
			DAQmxErrorLibraryNotPresent = -200090, 
			DAQmxErrorDuplicateTask = -200089, 
			DAQmxErrorInvalidTask = -200088, 
			DAQmxErrorInvalidChannel = -200087, 
			DAQmxErrorInvalidSyntaxForPhysicalChannelRange = -200086, 
			DAQmxErrorMinNotLessThanMax = -200082, 
			DAQmxErrorSampleRateNumChansConvertPeriodCombo = -200081, 
			DAQmxErrorAODuringCounter1DMAConflict = -200079, 
			DAQmxErrorAIDuringCounter0DMAConflict = -200078, 
			DAQmxErrorInvalidAttributeValue = -200077, 
			DAQmxErrorSuppliedCurrentDataOutsideSpecifiedRange = -200076, 
			DAQmxErrorSuppliedVoltageDataOutsideSpecifiedRange = -200075, 
			DAQmxErrorCannotStoreCalConst = -200074, 
			DAQmxErrorSCXIModuleNotFound = -200073, 
			DAQmxErrorDuplicatePhysicalChansNotSupported = -200072, 
			DAQmxErrorTooManyPhysicalChansInList = -200071, 
			DAQmxErrorInvalidAdvanceEventTriggerType = -200070, 
			DAQmxErrorDeviceIsNotAValidSwitch = -200069, 
			DAQmxErrorDeviceDoesNotSupportScanning = -200068, 
			DAQmxErrorScanListCannotBeTimed = -200067, 
			DAQmxErrorConnectOperatorInvalidAtPointInList = -200066, 
			DAQmxErrorUnexpectedSwitchActionInList = -200065, 
			DAQmxErrorUnexpectedSeparatorInList = -200064, 
			DAQmxErrorExpectedTerminatorInList = -200063, 
			DAQmxErrorExpectedConnectOperatorInList = -200062, 
			DAQmxErrorExpectedSeparatorInList = -200061, 
			DAQmxErrorFullySpecifiedPathInListContainsRange = -200060, 
			DAQmxErrorConnectionSeparatorAtEndOfList = -200059, 
			DAQmxErrorIdentifierInListTooLong = -200058, 
			DAQmxErrorDuplicateDeviceIDInListWhenSettling = -200057, 
			DAQmxErrorChannelNameNotSpecifiedInList = -200056, 
			DAQmxErrorDeviceIDNotSpecifiedInList = -200055, 
			DAQmxErrorSemicolonDoesNotFollowRangeInList = -200054, 
			DAQmxErrorSwitchActionInListSpansMultipleDevices = -200053, 
			DAQmxErrorRangeWithoutAConnectActionInList = -200052, 
			DAQmxErrorInvalidIdentifierFollowingSeparatorInList = -200051, 
			DAQmxErrorInvalidChannelNameInList = -200050, 
			DAQmxErrorInvalidNumberInRepeatStatementInList = -200049, 
			DAQmxErrorInvalidTriggerLineInList = -200048, 
			DAQmxErrorInvalidIdentifierInListFollowingDeviceID = -200047, 
			DAQmxErrorInvalidIdentifierInListAtEndOfSwitchAction = -200046, 
			DAQmxErrorDeviceRemoved = -200045, 
			DAQmxErrorRoutingPathNotAvailable = -200044, 
			DAQmxErrorRoutingHardwareBusy = -200043, 
			DAQmxErrorRequestedSignalInversionForRoutingNotPossible = -200042, 
			DAQmxErrorInvalidRoutingDestinationTerminalName = -200041, 
			DAQmxErrorInvalidRoutingSourceTerminalName = -200040, 
			DAQmxErrorRoutingNotSupportedForDevice = -200039, 
			DAQmxErrorWaitIsLastInstructionOfLoopInScript = -200038, 
			DAQmxErrorClearIsLastInstructionOfLoopInScript = -200037, 
			DAQmxErrorInvalidLoopIterationsInScript = -200036, 
			DAQmxErrorRepeatLoopNestingTooDeepInScript = -200035, 
			DAQmxErrorMarkerPositionOutsideSubsetInScript = -200034, 
			DAQmxErrorSubsetStartOffsetNotAlignedInScript = -200033, 
			DAQmxErrorInvalidSubsetLengthInScript = -200032, 
			DAQmxErrorMarkerPositionNotAlignedInScript = -200031, 
			DAQmxErrorSubsetOutsideWaveformInScript = -200030, 
			DAQmxErrorMarkerOutsideWaveformInScript = -200029, 
			DAQmxErrorWaveformInScriptNotInMem = -200028, 
			DAQmxErrorKeywordExpectedInScript = -200027, 
			DAQmxErrorBufferNameExpectedInScript = -200026, 
			DAQmxErrorProcedureNameExpectedInScript = -200025, 
			DAQmxErrorScriptHasInvalidIdentifier = -200024, 
			DAQmxErrorScriptHasInvalidCharacter = -200023, 
			DAQmxErrorResourceAlreadyReserved = -200022, 
			DAQmxErrorSelfTestFailed = -200020, 
			DAQmxErrorADCOverrun = -200019, 
			DAQmxErrorDACUnderflow = -200018, 
			DAQmxErrorInputFIFOUnderflow = -200017, 
			DAQmxErrorOutputFIFOUnderflow = -200016, 
			DAQmxErrorSCXISerialCommunication = -200015, 
			DAQmxErrorDigitalTerminalSpecifiedMoreThanOnce = -200014, 
			DAQmxErrorDigitalOutputNotSupported = -200012, 
			DAQmxErrorInconsistentChannelDirections = -200011, 
			DAQmxErrorInputFIFOOverflow = -200010, 
			DAQmxErrorTimeStampOverwritten = -200009, 
			DAQmxErrorStopTriggerHasNotOccurred = -200008, 
			DAQmxErrorRecordNotAvailable = -200007, 
			DAQmxErrorRecordOverwritten = -200006, 
			DAQmxErrorDataNotAvailable = -200005, 
			DAQmxErrorDataOverwrittenInDeviceMemory = -200004, 
			DAQmxErrorDuplicatedChannel = -200003, 
			DAQmxWarningTimestampCounterRolledOver = 200003, 
			DAQmxWarningInputTerminationOverloaded = 200004, 
			DAQmxWarningADCOverloaded = 200005, 
			DAQmxWarningPLLUnlocked = 200007, 
			DAQmxWarningCounter0DMADuringAIConflict = 200008, 
			DAQmxWarningCounter1DMADuringAOConflict = 200009, 
			DAQmxWarningStoppedBeforeDone = 200010, 
			DAQmxWarningRateViolatesSettlingTime = 200011, 
			DAQmxWarningRateViolatesMaxADCRate = 200012, 
			DAQmxWarningUserDefInfoStringTooLong = 200013, 
			DAQmxWarningTooManyInterruptsPerSecond = 200014, 
			DAQmxWarningPotentialGlitchDuringWrite = 200015, 
			DAQmxWarningDevNotSelfCalibratedWithDAQmx = 200016, 
			DAQmxWarningAISampRateTooLow = 200017, 
			DAQmxWarningAIConvRateTooLow = 200018, 
			DAQmxWarningReadOffsetCoercion = 200019, 
			DAQmxWarningPretrigCoercion = 200020, 
			DAQmxWarningSampValCoercedToMax = 200021, 
			DAQmxWarningSampValCoercedToMin = 200022, 
			DAQmxWarningPropertyVersionNew = 200024, 
			DAQmxWarningUserDefinedInfoTooLong = 200025, 
			DAQmxWarningCAPIStringTruncatedToFitBuffer = 200026, 
			DAQmxWarningSampClkRateTooLow = 200027, 
			DAQmxWarningPossiblyInvalidCTRSampsInFiniteDMAAcq = 200028, 
			DAQmxWarningRISAcqCompletedSomeBinsNotFilled = 200029, 
			DAQmxWarningPXIDevTempExceedsMaxOpTemp = 200030, 
			DAQmxWarningOutputGainTooLowForRFFreq = 200031, 
			DAQmxWarningOutputGainTooHighForRFFreq = 200032, 
			DAQmxWarningMultipleWritesBetweenSampClks = 200033, 
			DAQmxWarningDeviceMayShutDownDueToHighTemp = 200034, 
			DAQmxWarningReadNotCompleteBeforeSampClk = 209800, 
			DAQmxWarningWriteNotCompleteBeforeSampClk = 209801, 
		}
		private {/*Task Configuration/Control}*/
			extern (C) int function (const char* taskName, TaskHandle* taskHandle) DAQmxBaseLoadTask;
			extern (C) int function (const char* taskName, TaskHandle* taskHandle) DAQmxBaseCreateTask;
			extern (C) int function (TaskHandle taskHandle) DAQmxBaseStartTask;
			extern (C) int function (TaskHandle taskHandle) DAQmxBaseStopTask;
			extern (C) int function (TaskHandle taskHandle) DAQmxBaseClearTask;
			extern (C) int function (TaskHandle taskHandle, NIBool* isTaskDone) DAQmxBaseIsTaskDone;
		}
		private {/*Channel Configuration/Creation}*/
			extern (C) int function (TaskHandle taskHandle, const char* physicalChannel, const char* nameToAssignToChannel, int terminalConfig, double minVal, double maxVal, int units, const char* customScaleName) DAQmxBaseCreateAIVoltageChan;
			extern (C) int function (TaskHandle taskHandle, const char* physicalChannel, const char* nameToAssignToChannel, double minVal, double maxVal, int units, int thermocoupleType, int cjcSource, double cjcVal, const char* cjcChannel) DAQmxBaseCreateAIThrmcplChan;
			extern (C) int function (TaskHandle taskHandle, const char* physicalChannel, const char* nameToAssignToChannel, double minVal, double maxVal, int units, const char* customScaleName) DAQmxBaseCreateAOVoltageChan;
			extern (C) int function (TaskHandle taskHandle, const char* lines, const char* nameToAssignToLines, int lineGrouping) DAQmxBaseCreateDIChan;
			extern (C) int function (TaskHandle taskHandle, const char* lines, const char* nameToAssignToLines, int lineGrouping) DAQmxBaseCreateDOChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, double minVal, double maxVal, int units, int edge, int measMethod, double measTime, uint divisor, const char* customScaleName) DAQmxBaseCreateCIPeriodChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, int edge, uint initialCount, int countDirection) DAQmxBaseCreateCICountEdgesChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, double minVal, double maxVal, int units, int startingEdge, const char* customScaleName) DAQmxBaseCreateCIPulseWidthChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, int decodingType, NIBool ZidxEnable, double ZidxVal, int ZidxPhase, int units, double distPerPulse, double initialPos, const char* customScaleName) DAQmxBaseCreateCILinEncoderChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, int decodingType, NIBool ZidxEnable, double ZidxVal, int ZidxPhase, int units, uint pulsesPerRev, double initialAngle, const char* customScaleName) DAQmxBaseCreateCIAngEncoderChan;
			extern (C) int function (TaskHandle taskHandle, const char* counter, const char* nameToAssignToChannel, int units, int idleState, double initialDelay, double freq, double dutyCycle) DAQmxBaseCreateCOPulseChanFreq;
			extern (C) int function (TaskHandle taskHandle, const char* channel, int attribute, void* value) DAQmxBaseGetChanAttribute;
			extern (C) int function (TaskHandle taskHandle, const char* channel, int attribute, int value) DAQmxBaseSetChanAttribute;
		}
		private {/*Timing}*/
			extern (C) int function (TaskHandle taskHandle, const char* source, double rate, int activeEdge, int sampleMode, ulong sampsPerChan) DAQmxBaseCfgSampClkTiming;
			extern (C) int function (TaskHandle taskHandle, int sampleMode, ulong sampsPerChan) DAQmxBaseCfgImplicitTiming;
		}
		private {/*Triggering}*/
			extern (C) int function (TaskHandle taskHandle) DAQmxBaseDisableStartTrig;
			extern (C) int function (TaskHandle taskHandle, const char* triggerSource, int triggerEdge) DAQmxBaseCfgDigEdgeStartTrig;
			extern (C) int function (TaskHandle taskHandle, const char* triggerSource, int triggerSlope, double triggerLevel) DAQmxBaseCfgAnlgEdgeStartTrig;
			extern (C) int function (TaskHandle taskHandle) DAQmxBaseDisableRefTrig;
			extern (C) int function (TaskHandle taskHandle, const char* triggerSource, int triggerEdge, uint pretriggerSamples) DAQmxBaseCfgDigEdgeRefTrig;
			extern (C) int function (TaskHandle taskHandle, const char* triggerSource, int triggerSlope, double triggerLevel, uint pretriggerSamples) DAQmxBaseCfgAnlgEdgeRefTrig;
		}
		private {/*Read Data}*/
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, NIBool fillMode, double* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadAnalogF64;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, NIBool fillMode, short* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadBinaryI16;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, NIBool fillMode, int* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadBinaryI32;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, NIBool fillMode, ubyte* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadDigitalU8;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, NIBool fillMode, uint* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadDigitalU32;
			extern (C) int function (TaskHandle taskHandle, double timeout, uint* value, NIBool* reserved) DAQmxBaseReadDigitalScalarU32;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, double* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadCounterF64;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, double timeout, uint* readArray, uint arraySizeInSamps, int* sampsPerChanRead, NIBool* reserved) DAQmxBaseReadCounterU32;
			extern (C) int function (TaskHandle taskHandle, double timeout, double* value, NIBool* reserved) DAQmxBaseReadCounterScalarF64;
			extern (C) int function (TaskHandle taskHandle, double timeout, uint* value, NIBool* reserved) DAQmxBaseReadCounterScalarU32;
			extern (C) int function (TaskHandle taskHandle, int attribute, void* value) DAQmxBaseGetReadAttribute;
		}
		private {/*Write Data}*/
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, NIBool autoStart, double timeout, NIBool dataLayout, double* writeArray, int* sampsPerChanWritten, NIBool* reserved) DAQmxBaseWriteAnalogF64;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, NIBool autoStart, double timeout, NIBool dataLayout, ubyte* writeArray, int* sampsPerChanWritten, NIBool* reserved) DAQmxBaseWriteDigitalU8;
			extern (C) int function (TaskHandle taskHandle, int numSampsPerChan, NIBool autoStart, double timeout, NIBool dataLayout, uint* writeArray, int* sampsPerChanWritten, NIBool* reserved) DAQmxBaseWriteDigitalU32;
			extern (C) int function (TaskHandle taskHandle, NIBool autoStart, double timeout, uint value, NIBool* reserved) DAQmxBaseWriteDigitalScalarU32;
			extern (C) int function (TaskHandle taskHandle, int attribute, void* value) DAQmxBaseGetWriteAttribute;
			extern (C) int function (TaskHandle taskHandle, int attribute, int value) DAQmxBaseSetWriteAttribute;
		}
		private {/*Events & Signals}*/
			extern (C) int function (TaskHandle taskHandle, int signalID, const char* outputTerminal) DAQmxBaseExportSignal;
		}
		private {/*Buffer Configurations}*/
			extern (C) int function (TaskHandle taskHandle, uint numSampsPerChan) DAQmxBaseCfgInputBuffer;
		}
		private {/*Device Control}*/
			extern (C) int function (const char* deviceName) DAQmxBaseResetDevice;
		}
		private {/*Error Handling}*/
			extern (C) int function (char* errorString, uint bufferSize) DAQmxBaseGetExtendedErrorInfo;
		}
		private {/*NI-DAQmxBase Specific Attribute Get/Set/Reset Function Declarations}*/
			extern (C) int function (const char* device, uint* data) DAQmxBaseGetDevSerialNum;
		}

		static:
		mixin DynamicLibrary;
		shared static this () {load_library;}
}

public {/*NI-DAQmx Attributes}*/
	enum {/*Custom Scale}*/
		DAQmx_Val_FromCustomScale = 10065, // From Custom Scale
	}
	enum {/*Calibration Info Attributes}*/
		DAQmx_SelfCal_Supported = 0x1860, // Indicates whether the device supports self calibration.
		DAQmx_SelfCal_LastTemp = 0x1864, // Indicates in degrees Celsius the temperature of the device at the time of the last self calibration. Compare this temperature to the current onboard temperature to determine if you should perform another calibration.
		DAQmx_ExtCal_RecommendedInterval = 0x1868, // Indicates in months the National Instruments recommended interval between each external calibration of the device.
		DAQmx_ExtCal_LastTemp = 0x1867, // Indicates in degrees Celsius the temperature of the device at the time of the last external calibration. Compare this temperature to the current onboard temperature to determine if you should perform another calibration.
		DAQmx_Cal_UserDefinedInfo = 0x1861, // Specifies a string that contains arbitrary, user-defined information. This number of characters in this string can be no more than Max Size.
		DAQmx_Cal_UserDefinedInfo_MaxSize = 0x191C, // Indicates the maximum length in characters of Information.
	}
	enum {/*Channel Attributes}*/
		DAQmx_ChanType = 0x187F, // Indicates the type of the virtual channel.
		DAQmx_PhysicalChanName = 0x18F5, // Indicates the name of the physical channel upon which this virtual channel is based.
		DAQmx_ChanDescr = 0x1926, // Specifies a user-defined description for the channel.
		DAQmx_AI_Max = 0x17DD, // Specifies the maximum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced maximum value that the device can measure with the current settings.
		DAQmx_AI_Min = 0x17DE, // Specifies the minimum value you expect to measure. This value is in the units you specify with a units property.  When you query this property, it returns the coerced minimum value that the device can measure with the current settings.
		DAQmx_AI_CustomScaleName = 0x17E0, // Specifies the name of a custom scale for the channel.
		DAQmx_AI_MeasType = 0x0695, // Indicates the measurement to take with the analog input channel and in some cases, such as for temperature measurements, the sensor to use.
		DAQmx_AI_Voltage_Units = 0x1094, // Specifies the units to use to return voltage measurements from the channel.
		DAQmx_AI_Temp_Units = 0x1033, // Specifies the units to use to return temperature measurements from the channel.
		DAQmx_AI_Thrmcpl_Type = 0x1050, // Specifies the type of thermocouple connected to the channel. Thermocouple types differ in composition and measurement range.
		DAQmx_AI_Thrmcpl_CJCSrc = 0x1035, // Indicates the source of cold-junction compensation.
		DAQmx_AI_Thrmcpl_CJCVal = 0x1036, // Specifies the temperature of the cold junction if CJC Source is DAQmx_Val_ConstVal. Specify this value in the units of the measurement.
		DAQmx_AI_Thrmcpl_CJCChan = 0x1034, // Indicates the channel that acquires the temperature of the cold junction if CJC Source is DAQmx_Val_Chan. If the channel does not use a custom scale, NI-DAQmx uses the correct units. If the channel uses a custom scale, the pre-scaled units of the channel must be degrees Celsius.
		DAQmx_AI_RTD_Type = 0x1032, // Specifies the type of RTD connected to the channel.
		DAQmx_AI_RTD_R0 = 0x1030, // Specifies in ohms the sensor resistance at 0 deg C. The Callendar-Van Dusen equation requires this value. Refer to the sensor documentation to determine this value.
		DAQmx_AI_RTD_A = 0x1010, // Specifies the 'A' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
		DAQmx_AI_RTD_B = 0x1011, // Specifies the 'B' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
		DAQmx_AI_RTD_C = 0x1013, // Specifies the 'C' constant of the Callendar-Van Dusen equation. NI-DAQmx requires this value when you use a custom RTD.
		DAQmx_AI_Thrmstr_A = 0x18C9, // Specifies the 'A' constant of the Steinhart-Hart thermistor equation.
		DAQmx_AI_Thrmstr_B = 0x18CB, // Specifies the 'B' constant of the Steinhart-Hart thermistor equation.
		DAQmx_AI_Thrmstr_C = 0x18CA, // Specifies the 'C' constant of the Steinhart-Hart thermistor equation.
		DAQmx_AI_Thrmstr_R1 = 0x1061, // Specifies in ohms the value of the reference resistor if you use voltage excitation. NI-DAQmx ignores this value for current excitation.
		DAQmx_AI_ForceReadFromChan = 0x18F8, // Specifies whether to read from the channel if it is a cold-junction compensation channel. By default, an NI-DAQmx Read function does not return data from cold-junction compensation channels.  Setting this property to TRUE forces read operations to return the cold-junction compensation channel data with the other channels in the task.
		DAQmx_AI_Current_Units = 0x0701, // Specifies the units to use to return current measurements from the channel.
		DAQmx_AI_Strain_Units = 0x0981, // Specifies the units to use to return strain measurements from the channel.
		DAQmx_AI_StrainGage_GageFactor = 0x0994, // Specifies the sensitivity of the strain gage.  Gage factor relates the change in electrical resistance to the change in strain. Refer to the sensor documentation for this value.
		DAQmx_AI_StrainGage_PoissonRatio = 0x0998, // Specifies the ratio of lateral strain to axial strain in the material you are measuring.
		DAQmx_AI_StrainGage_Cfg = 0x0982, // Specifies the bridge configuration of the strain gages.
		DAQmx_AI_Resistance_Units = 0x0955, // Specifies the units to use to return resistance measurements.
		DAQmx_AI_Freq_Units = 0x0806, // Specifies the units to use to return frequency measurements from the channel.
		DAQmx_AI_Freq_ThreshVoltage = 0x0815, // Specifies the voltage level at which to recognize waveform repetitions. You should select a voltage level that occurs only once within the entire period of a waveform. You also can select a voltage that occurs only once while the voltage rises or falls.
		DAQmx_AI_Freq_Hyst = 0x0814, // Specifies in volts a window below Threshold Level. The input voltage must pass below Threshold Level minus this value before NI-DAQmx recognizes a waveform repetition at Threshold Level. Hysteresis can improve the measurement accuracy when the signal contains noise or jitter.
		DAQmx_AI_LVDT_Units = 0x0910, // Specifies the units to use to return linear position measurements from the channel.
		DAQmx_AI_LVDT_Sensitivity = 0x0939, // Specifies the sensitivity of the LVDT. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
		DAQmx_AI_LVDT_SensitivityUnits = 0x219A, // Specifies the units of Sensitivity.
		DAQmx_AI_RVDT_Units = 0x0877, // Specifies the units to use to return angular position measurements from the channel.
		DAQmx_AI_RVDT_Sensitivity = 0x0903, // Specifies the sensitivity of the RVDT. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
		DAQmx_AI_RVDT_SensitivityUnits = 0x219B, // Specifies the units of Sensitivity.
		DAQmx_AI_Accel_Units = 0x0673, // Specifies the units to use to return acceleration measurements from the channel.
		DAQmx_AI_Accel_Sensitivity = 0x0692, // Specifies the sensitivity of the accelerometer. This value is in the units you specify with Sensitivity Units. Refer to the sensor documentation to determine this value.
		DAQmx_AI_Accel_SensitivityUnits = 0x219C, // Specifies the units of Sensitivity.
		DAQmx_AI_Coupling = 0x0064, // Specifies the coupling for the channel.
		DAQmx_AI_Impedance = 0x0062, // Specifies the input impedance of the channel.
		DAQmx_AI_TermCfg = 0x1097, // Specifies the terminal configuration for the channel.
		DAQmx_AI_ResistanceCfg = 0x1881, // Specifies the resistance configuration for the channel. NI-DAQmx uses this value for any resistance-based measurements, including temperature measurement using a thermistor or RTD.
		DAQmx_AI_LeadWireResistance = 0x17EE, // Specifies in ohms the resistance of the wires that lead to the sensor.
		DAQmx_AI_Bridge_Cfg = 0x0087, // Specifies the type of Wheatstone bridge that the sensor is.
		DAQmx_AI_Bridge_NomResistance = 0x17EC, // Specifies in ohms the resistance across each arm of the bridge in an unloaded position.
		DAQmx_AI_Bridge_InitialVoltage = 0x17ED, // Specifies in volts the output voltage of the bridge in the unloaded condition. NI-DAQmx subtracts this value from any measurements before applying scaling equations.
		DAQmx_AI_Bridge_ShuntCal_Enable = 0x0094, // Specifies whether to enable a shunt calibration switch. Use Shunt Cal Select to select the switch(es) to enable.
		DAQmx_AI_Bridge_ShuntCal_Select = 0x21D5, // Specifies which shunt calibration switch(es) to enable.  Use Shunt Cal Enable to enable the switch(es) you specify with this property.
		DAQmx_AI_Bridge_ShuntCal_GainAdjust = 0x193F, // Specifies the result of a shunt calibration. NI-DAQmx multiplies data read from the channel by the value of this property. This value should be close to 1.0.
		DAQmx_AI_Bridge_Balance_CoarsePot = 0x17F1, // Specifies by how much to compensate for offset in the signal. This value can be between 0 and 127.
		DAQmx_AI_Bridge_Balance_FinePot = 0x18F4, // Specifies by how much to compensate for offset in the signal. This value can be between 0 and 4095.
		DAQmx_AI_CurrentShunt_Loc = 0x17F2, // Specifies the shunt resistor location for current measurements.
		DAQmx_AI_CurrentShunt_Resistance = 0x17F3, // Specifies in ohms the external shunt resistance for current measurements.
		DAQmx_AI_Excit_Src = 0x17F4, // Specifies the source of excitation.
		DAQmx_AI_Excit_Val = 0x17F5, // Specifies the amount of excitation that the sensor requires. If Voltage or Current is  DAQmx_Val_Voltage, this value is in volts. If Voltage or Current is  DAQmx_Val_Current, this value is in amperes.
		DAQmx_AI_Excit_UseForScaling = 0x17FC, // Specifies if NI-DAQmx divides the measurement by the excitation. You should typically set this property to TRUE for ratiometric transducers. If you set this property to TRUE, set Maximum Value and Minimum Value to reflect the scaling.
		DAQmx_AI_Excit_UseMultiplexed = 0x2180, // Specifies if the SCXI-1122 multiplexes the excitation to the upper half of the channels as it advances through the scan list.
		DAQmx_AI_Excit_ActualVal = 0x1883, // Specifies the actual amount of excitation supplied by an internal excitation source.  If you read an internal excitation source more precisely with an external device, set this property to the value you read.  NI-DAQmx ignores this value for external excitation.
		DAQmx_AI_Excit_DCorAC = 0x17FB, // Specifies if the excitation supply is DC or AC.
		DAQmx_AI_Excit_VoltageOrCurrent = 0x17F6, // Specifies if the channel uses current or voltage excitation.
		DAQmx_AI_ACExcit_Freq = 0x0101, // Specifies the AC excitation frequency in Hertz.
		DAQmx_AI_ACExcit_SyncEnable = 0x0102, // Specifies whether to synchronize the AC excitation source of the channel to that of another channel. Synchronize the excitation sources of multiple channels to use multichannel sensors. Set this property to FALSE for the master channel and to TRUE for the slave channels.
		DAQmx_AI_ACExcit_WireMode = 0x18CD, // Specifies the number of leads on the LVDT or RVDT. Some sensors require you to tie leads together to create a four- or five- wire sensor. Refer to the sensor documentation for more information.
		DAQmx_AI_Atten = 0x1801, // Specifies the amount of attenuation to use.
		DAQmx_AI_Lowpass_Enable = 0x1802, // Specifies whether to enable the lowpass filter of the channel.
		DAQmx_AI_Lowpass_CutoffFreq = 0x1803, // Specifies the frequency in Hertz that corresponds to the -3dB cutoff of the filter.
		DAQmx_AI_Lowpass_SwitchCap_ClkSrc = 0x1884, // Specifies the source of the filter clock. If you need a higher resolution for the filter, you can supply an external clock to increase the resolution. Refer to the SCXI-1141/1142/1143 User Manual for more information.
		DAQmx_AI_Lowpass_SwitchCap_ExtClkFreq = 0x1885, // Specifies the frequency of the external clock when you set Clock Source to DAQmx_Val_External.  NI-DAQmx uses this frequency to set the pre- and post- filters on the SCXI-1141, SCXI-1142, and SCXI-1143. On those devices, NI-DAQmx determines the filter cutoff by using the equation f/(100*n), where f is the external frequency, and n is the external clock divisor. Refer to the SCXI-1141/1142/1143 User Manual for more...
		DAQmx_AI_Lowpass_SwitchCap_ExtClkDiv = 0x1886, // Specifies the divisor for the external clock when you set Clock Source to DAQmx_Val_External. On the SCXI-1141, SCXI-1142, and SCXI-1143, NI-DAQmx determines the filter cutoff by using the equation f/(100*n), where f is the external frequency, and n is the external clock divisor. Refer to the SCXI-1141/1142/1143 User Manual for more information.
		DAQmx_AI_Lowpass_SwitchCap_OutClkDiv = 0x1887, // Specifies the divisor for the output clock.  NI-DAQmx uses the cutoff frequency to determine the output clock frequency. Refer to the SCXI-1141/1142/1143 User Manual for more information.
		DAQmx_AI_ResolutionUnits = 0x1764, // Indicates the units of Resolution Value.
		DAQmx_AI_Resolution = 0x1765, // Indicates the resolution of the analog-to-digital converter of the channel. This value is in the units you specify with Resolution Units.
		DAQmx_AI_Dither_Enable = 0x0068, // Specifies whether to enable dithering.  Dithering adds Gaussian noise to the input signal. You can use dithering to achieve higher resolution measurements by over sampling the input signal and averaging the results.
		DAQmx_AI_Rng_High = 0x1815, // Specifies the upper limit of the input range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
		DAQmx_AI_Rng_Low = 0x1816, // Specifies the lower limit of the input range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
		DAQmx_AI_Gain = 0x1818, // Specifies a gain factor to apply to the channel.
		DAQmx_AI_SampAndHold_Enable = 0x181A, // Specifies whether to enable the sample and hold circuitry of the device. When you disable sample and hold circuitry, a small voltage offset might be introduced into the signal.  You can eliminate this offset by using Auto Zero Mode to perform an auto zero on the channel.
		DAQmx_AI_AutoZeroMode = 0x1760, // Specifies when to measure ground. NI-DAQmx subtracts the measured ground voltage from every sample.
		DAQmx_AI_DataXferMech = 0x1821, // Specifies the data transfer mode for the device.
		DAQmx_AI_DataXferReqCond = 0x188B, // Specifies under what condition to transfer data from the onboard memory of the device to the buffer.
		DAQmx_AI_MemMapEnable = 0x188C, // Specifies for NI-DAQmx to map hardware registers to the memory space of the customer process, if possible. Mapping to the memory space of the customer process increases performance. However, memory mapping can adversely affect the operation of the device and possibly result in a system crash if software in the process unintentionally accesses the mapped registers.
		DAQmx_AI_DevScalingCoeff = 0x1930, // Indicates the coefficients of a polynomial equation that NI-DAQmx uses to scale values from the native format of the device to volts. Each element of the array corresponds to a term of the equation. For example, if index two of the array is 4, the third term of the equation is 4x^2. Scaling coefficients do not account for any custom scales or sensors contained by the channel.
		DAQmx_AO_Max = 0x1186, // Specifies the maximum value you expect to generate. The value is in the units you specify with a units property. If you try to write a value larger than the maximum value, NI-DAQmx generates an error. NI-DAQmx might coerce this value to a smaller value if other task settings restrict the device from generating the desired maximum.
		DAQmx_AO_Min = 0x1187, // Specifies the minimum value you expect to generate. The value is in the units you specify with a units property. If you try to write a value smaller than the minimum value, NI-DAQmx generates an error. NI-DAQmx might coerce this value to a larger value if other task settings restrict the device from generating the desired minimum.
		DAQmx_AO_CustomScaleName = 0x1188, // Specifies the name of a custom scale for the channel.
		DAQmx_AO_OutputType = 0x1108, // Indicates whether the channel generates voltage or current.
		DAQmx_AO_Voltage_Units = 0x1184, // Specifies in what units to generate voltage on the channel. Write data to the channel in the units you select.
		DAQmx_AO_Current_Units = 0x1109, // Specifies in what units to generate current on the channel. Write data to the channel is in the units you select.
		DAQmx_AO_OutputImpedance = 0x1490, // Specifies in ohms the impedance of the analog output stage of the device.
		DAQmx_AO_LoadImpedance = 0x0121, // Specifies in ohms the load impedance connected to the analog output channel.
		DAQmx_AO_ResolutionUnits = 0x182B, // Specifies the units of Resolution Value.
		DAQmx_AO_Resolution = 0x182C, // Indicates the resolution of the digital-to-analog converter of the channel. This value is in the units you specify with Resolution Units.
		DAQmx_AO_DAC_Rng_High = 0x182E, // Specifies the upper limit of the output range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
		DAQmx_AO_DAC_Rng_Low = 0x182D, // Specifies the lower limit of the output range of the device. This value is in the native units of the device. On E Series devices, for example, the native units is volts.
		DAQmx_AO_DAC_Ref_ConnToGnd = 0x0130, // Specifies whether to ground the internal DAC reference. Grounding the internal DAC reference has the effect of grounding all analog output channels and stopping waveform generation across all analog output channels regardless of whether the channels belong to the current task. You can ground the internal DAC reference only when Source is DAQmx_Val_Internal and Allow Connecting DAC Reference to Ground at Runtime is...
		DAQmx_AO_DAC_Ref_AllowConnToGnd = 0x1830, // Specifies whether to allow grounding the internal DAC reference at run time. You must set this property to TRUE and set Source to DAQmx_Val_Internal before you can set Connect DAC Reference to Ground to TRUE.
		DAQmx_AO_DAC_Ref_Src = 0x0132, // Specifies the source of the DAC reference voltage.  The value of this voltage source determines the full-scale value of the DAC.
		DAQmx_AO_DAC_Ref_Val = 0x1832, // Specifies in volts the value of the DAC reference voltage. This voltage determines the full-scale range of the DAC. Smaller reference voltages result in smaller ranges, but increased resolution.
		DAQmx_AO_ReglitchEnable = 0x0133, // Specifies whether to enable reglitching.  The output of a DAC normally glitches whenever the DAC is updated with a new value. The amount of glitching differs from code to code and is generally largest at major code transitions.  Reglitching generates uniform glitch energy at each code transition and provides for more uniform glitches.  Uniform glitch energy makes it easier to filter out the noise introduced from g...
		DAQmx_AO_UseOnlyOnBrdMem = 0x183A, // Specifies whether to write samples directly to the onboard memory of the device, bypassing the memory buffer. Generally, you cannot update onboard memory after you start the task. Onboard memory includes data FIFOs.
		DAQmx_AO_DataXferMech = 0x0134, // Specifies the data transfer mode for the device.
		DAQmx_AO_DataXferReqCond = 0x183C, // Specifies under what condition to transfer data from the buffer to the onboard memory of the device.
		DAQmx_AO_MemMapEnable = 0x188F, // Specifies if NI-DAQmx maps hardware registers to the memory space of the customer process, if possible. Mapping to the memory space of the customer process increases performance. However, memory mapping can adversely affect the operation of the device and possibly result in a system crash if software in the process unintentionally accesses the mapped registers.
		DAQmx_AO_DevScalingCoeff = 0x1931, // Indicates the coefficients of a linear equation that NI-DAQmx uses to scale values from a voltage to the native format of the device.  Each element of the array corresponds to a term of the equation. For example, if index two of the array is 4, the third term of the equation is 4x^2.  Scaling coefficients do not account for any custom scales that may be applied to the channel.
		DAQmx_DI_InvertLines = 0x0793, // Specifies whether to invert the lines in the channel. If you set this property to TRUE, the lines are at high logic when off and at low logic when on.
		DAQmx_DI_NumLines = 0x2178, // Indicates the number of digital lines in the channel.
		DAQmx_DI_DigFltr_Enable = 0x21D6, // Specifies whether to enable the digital filter for the line(s) or port(s). You can enable the filter on a line-by-line basis. You do not have to enable the filter for all lines in a channel.
		DAQmx_DI_DigFltr_MinPulseWidth = 0x21D7, // Specifies in seconds the minimum pulse width the filter recognizes as a valid high or low state transition.
		DAQmx_DO_InvertLines = 0x1133, // Specifies whether to invert the lines in the channel. If you set this property to TRUE, the lines are at high logic when off and at low logic when on.
		DAQmx_DO_NumLines = 0x2179, // Indicates the number of digital lines in the channel.
		DAQmx_DO_Tristate = 0x18F3, // Specifies whether to stop driving the channel and set it to a Hi-Z state.
		DAQmx_CI_Max = 0x189C, // Specifies the maximum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced maximum value that the hardware can measure with the current settings.
		DAQmx_CI_Min = 0x189D, // Specifies the minimum value you expect to measure. This value is in the units you specify with a units property. When you query this property, it returns the coerced minimum value that the hardware can measure with the current settings.
		DAQmx_CI_CustomScaleName = 0x189E, // Specifies the name of a custom scale for the channel.
		DAQmx_CI_MeasType = 0x18A0, // Indicates the measurement to take with the channel.
		DAQmx_CI_Freq_Units = 0x18A1, // Specifies the units to use to return frequency measurements.
		DAQmx_CI_Freq_Term = 0x18A2, // Specifies the input terminal of the signal to measure.
		DAQmx_CI_Freq_StartingEdge = 0x0799, // Specifies between which edges to measure the frequency of the signal.
		DAQmx_CI_Freq_MeasMeth = 0x0144, // Specifies the method to use to measure the frequency of the signal.
		DAQmx_CI_Freq_MeasTime = 0x0145, // Specifies in seconds the length of time to measure the frequency of the signal if Method is DAQmx_Val_HighFreq2Ctr. Measurement accuracy increases with increased measurement time and with increased signal frequency. If you measure a high-frequency signal for too long, however, the count register could roll over, which results in an incorrect measurement.
		DAQmx_CI_Freq_Div = 0x0147, // Specifies the value by which to divide the input signal if  Method is DAQmx_Val_LargeRng2Ctr. The larger the divisor, the more accurate the measurement. However, too large a value could cause the count register to roll over, which results in an incorrect measurement.
		DAQmx_CI_Period_Units = 0x18A3, // Specifies the unit to use to return period measurements.
		DAQmx_CI_Period_Term = 0x18A4, // Specifies the input terminal of the signal to measure.
		DAQmx_CI_Period_StartingEdge = 0x0852, // Specifies between which edges to measure the period of the signal.
		DAQmx_CI_Period_MeasMeth = 0x192C, // Specifies the method to use to measure the period of the signal.
		DAQmx_CI_Period_MeasTime = 0x192D, // Specifies in seconds the length of time to measure the period of the signal if Method is DAQmx_Val_HighFreq2Ctr. Measurement accuracy increases with increased measurement time and with increased signal frequency. If you measure a high-frequency signal for too long, however, the count register could roll over, which results in an incorrect measurement.
		DAQmx_CI_Period_Div = 0x192E, // Specifies the value by which to divide the input signal if Method is DAQmx_Val_LargeRng2Ctr. The larger the divisor, the more accurate the measurement. However, too large a value could cause the count register to roll over, which results in an incorrect measurement.
		DAQmx_CI_CountEdges_Term = 0x18C7, // Specifies the input terminal of the signal to measure.
		DAQmx_CI_CountEdges_Dir = 0x0696, // Specifies whether to increment or decrement the counter on each edge.
		DAQmx_CI_CountEdges_DirTerm = 0x21E1, // Specifies the source terminal of the digital signal that controls the count direction if Direction is DAQmx_Val_ExtControlled.
		DAQmx_CI_CountEdges_InitialCnt = 0x0698, // Specifies the starting value from which to count.
		DAQmx_CI_CountEdges_ActiveEdge = 0x0697, // Specifies on which edges to increment or decrement the counter.
		DAQmx_CI_AngEncoder_Units = 0x18A6, // Specifies the units to use to return angular position measurements from the channel.
		DAQmx_CI_AngEncoder_PulsesPerRev = 0x0875, // Specifies the number of pulses the encoder generates per revolution. This value is the number of pulses on either signal A or signal B, not the total number of pulses on both signal A and signal B.
		DAQmx_CI_AngEncoder_InitialAngle = 0x0881, // Specifies the starting angle of the encoder. This value is in the units you specify with Units.
		DAQmx_CI_LinEncoder_Units = 0x18A9, // Specifies the units to use to return linear encoder measurements from the channel.
		DAQmx_CI_LinEncoder_DistPerPulse = 0x0911, // Specifies the distance to measure for each pulse the encoder generates on signal A or signal B. This value is in the units you specify with Units.
		DAQmx_CI_LinEncoder_InitialPos = 0x0915, // Specifies the position of the encoder when the measurement begins. This value is in the units you specify with Units.
		DAQmx_CI_Encoder_DecodingType = 0x21E6, // Specifies how to count and interpret the pulses the encoder generates on signal A and signal B. DAQmx_Val_X1, DAQmx_Val_X2, and DAQmx_Val_X4 are valid for quadrature encoders only. DAQmx_Val_TwoPulseCounting is valid for two-pulse encoders only.
		DAQmx_CI_Encoder_AInputTerm = 0x219D, // Specifies the terminal to which signal A is connected.
		DAQmx_CI_Encoder_BInputTerm = 0x219E, // Specifies the terminal to which signal B is connected.
		DAQmx_CI_Encoder_ZInputTerm = 0x219F, // Specifies the terminal to which signal Z is connected.
		DAQmx_CI_Encoder_ZIndexEnable = 0x0890, // Specifies whether to use Z indexing for the channel.
		DAQmx_CI_Encoder_ZIndexVal = 0x0888, // Specifies the value to which to reset the measurement when signal Z is high and signal A and signal B are at the states you specify with Z Index Phase. Specify this value in the units of the measurement.
		DAQmx_CI_Encoder_ZIndexPhase = 0x0889, // Specifies the states at which signal A and signal B must be while signal Z is high for NI-DAQmx to reset the measurement. If signal Z is never high while signal A and signal B are high, for example, you must choose a phase other than DAQmx_Val_AHighBHigh.
		DAQmx_CI_PulseWidth_Units = 0x0823, // Specifies the units to use to return pulse width measurements.
		DAQmx_CI_PulseWidth_Term = 0x18AA, // Specifies the input terminal of the signal to measure.
		DAQmx_CI_PulseWidth_StartingEdge = 0x0825, // Specifies on which edge of the input signal to begin each pulse width measurement.
		DAQmx_CI_TwoEdgeSep_Units = 0x18AC, // Specifies the units to use to return two-edge separation measurements from the channel.
		DAQmx_CI_TwoEdgeSep_FirstTerm = 0x18AD, // Specifies the source terminal of the digital signal that starts each measurement.
		DAQmx_CI_TwoEdgeSep_FirstEdge = 0x0833, // Specifies on which edge of the first signal to start each measurement.
		DAQmx_CI_TwoEdgeSep_SecondTerm = 0x18AE, // Specifies the source terminal of the digital signal that stops each measurement.
		DAQmx_CI_TwoEdgeSep_SecondEdge = 0x0834, // Specifies on which edge of the second signal to stop each measurement.
		DAQmx_CI_SemiPeriod_Units = 0x18AF, // Specifies the units to use to return semi-period measurements.
		DAQmx_CI_SemiPeriod_Term = 0x18B0, // Specifies the input terminal of the signal to measure.
		DAQmx_CI_CtrTimebaseSrc = 0x0143, // Specifies the terminal of the timebase to use for the counter.
		DAQmx_CI_CtrTimebaseRate = 0x18B2, // Specifies in Hertz the frequency of the counter timebase. Specifying the rate of a counter timebase allows you to take measurements in terms of time or frequency rather than in ticks of the timebase. If you use an external timebase and do not specify the rate, you can take measurements only in terms of ticks of the timebase.
		DAQmx_CI_CtrTimebaseActiveEdge = 0x0142, // Specifies whether a timebase cycle is from rising edge to rising edge or from falling edge to falling edge.
		DAQmx_CI_Count = 0x0148, // Indicates the current value of the count register.
		DAQmx_CI_OutputState = 0x0149, // Indicates the current state of the out terminal of the counter.
		DAQmx_CI_TCReached = 0x0150, // Indicates whether the counter rolled over. When you query this property, NI-DAQmx resets it to FALSE.
		DAQmx_CI_CtrTimebaseMasterTimebaseDiv = 0x18B3, // Specifies the divisor for an external counter timebase. You can divide the counter timebase in order to measure slower signals without causing the count register to roll over.
		DAQmx_CI_DataXferMech = 0x0200, // Specifies the data transfer mode for the channel.
		DAQmx_CI_NumPossiblyInvalidSamps = 0x193C, // Indicates the number of samples that the device might have overwritten before it could transfer them to the buffer.
		DAQmx_CI_DupCountPrevent = 0x21AC, // Specifies whether to enable duplicate count prevention for the channel.
		DAQmx_CO_OutputType = 0x18B5, // Indicates how to define pulses generated on the channel.
		DAQmx_CO_Pulse_IdleState = 0x1170, // Specifies the resting state of the output terminal.
		DAQmx_CO_Pulse_Term = 0x18E1, // Specifies on which terminal to generate pulses.
		DAQmx_CO_Pulse_Time_Units = 0x18D6, // Specifies the units in which to define high and low pulse time.
		DAQmx_CO_Pulse_HighTime = 0x18BA, // Specifies the amount of time that the pulse is at a high voltage. This value is in the units you specify with Units or when you create the channel.
		DAQmx_CO_Pulse_LowTime = 0x18BB, // Specifies the amount of time that the pulse is at a low voltage. This value is in the units you specify with Units or when you create the channel.
		DAQmx_CO_Pulse_Time_InitialDelay = 0x18BC, // Specifies in seconds the amount of time to wait before generating the first pulse.
		DAQmx_CO_Pulse_DutyCyc = 0x1176, // Specifies the duty cycle of the pulses. The duty cycle of a signal is the width of the pulse divided by period. NI-DAQmx uses this ratio and the pulse frequency to determine the width of the pulses and the delay between pulses.
		DAQmx_CO_Pulse_Freq_Units = 0x18D5, // Specifies the units in which to define pulse frequency.
		DAQmx_CO_Pulse_Freq = 0x1178, // Specifies the frequency of the pulses to generate. This value is in the units you specify with Units or when you create the channel.
		DAQmx_CO_Pulse_Freq_InitialDelay = 0x0299, // Specifies in seconds the amount of time to wait before generating the first pulse.
		DAQmx_CO_Pulse_HighTicks = 0x1169, // Specifies the number of ticks the pulse is high.
		DAQmx_CO_Pulse_LowTicks = 0x1171, // Specifies the number of ticks the pulse is low.
		DAQmx_CO_Pulse_Ticks_InitialDelay = 0x0298, // Specifies the number of ticks to wait before generating the first pulse.
		DAQmx_CO_CtrTimebaseSrc = 0x0339, // Specifies the terminal of the timebase to use for the counter. Typically, NI-DAQmx uses one of the internal counter timebases when generating pulses. Use this property to specify an external timebase and produce custom pulse widths that are not possible using the internal timebases.
		DAQmx_CO_CtrTimebaseRate = 0x18C2, // Specifies in Hertz the frequency of the counter timebase. Specifying the rate of a counter timebase allows you to define output pulses in seconds rather than in ticks of the timebase. If you use an external timebase and do not specify the rate, you can define output pulses only in ticks of the timebase.
		DAQmx_CO_CtrTimebaseActiveEdge = 0x0341, // Specifies whether a timebase cycle is from rising edge to rising edge or from falling edge to falling edge.
		DAQmx_CO_Count = 0x0293, // Indicates the current value of the count register.
		DAQmx_CO_OutputState = 0x0294, // Indicates the current state of the output terminal of the counter.
		DAQmx_CO_AutoIncrCnt = 0x0295, // Specifies a number of timebase ticks by which to increment each successive pulse.
		DAQmx_CO_CtrTimebaseMasterTimebaseDiv = 0x18C3, // Specifies the divisor for an external counter timebase. You can divide the counter timebase in order to generate slower signals without causing the count register to roll over.
		DAQmx_CO_PulseDone = 0x190E, // Indicates if the task completed pulse generation. Use this value for retriggerable pulse generation when you need to determine if the device generated the current pulse. When you query this property, NI-DAQmx resets it to FALSE.
	}
	enum {/*Export Signal Attributes}*/
		DAQmx_Exported_AIConvClk_OutputTerm = 0x1687, // Specifies the terminal to which to route the AI Convert Clock.
		DAQmx_Exported_AIConvClk_Pulse_Polarity = 0x1688, // Indicates the polarity of the exported AI Convert Clock. The polarity is fixed and independent of the active edge of the source of the AI Convert Clock.
		DAQmx_Exported_20MHzTimebase_OutputTerm = 0x1657, // Specifies the terminal to which to route the 20MHz Timebase.
		DAQmx_Exported_SampClk_OutputBehavior = 0x186B, // Specifies whether the exported Sample Clock issues a pulse at the beginning of a sample or changes to a high state for the duration of the sample.
		DAQmx_Exported_SampClk_OutputTerm = 0x1663, // Specifies the terminal to which to route the Sample Clock.
		DAQmx_Exported_AdvTrig_OutputTerm = 0x1645, // Specifies the terminal to which to route the Advance Trigger.
		DAQmx_Exported_AdvTrig_Pulse_Polarity = 0x1646, // Indicates the polarity of the exported Advance Trigger.
		DAQmx_Exported_AdvTrig_Pulse_WidthUnits = 0x1647, // Specifies the units of Width Value.
		DAQmx_Exported_AdvTrig_Pulse_Width = 0x1648, // Specifies the width of an exported Advance Trigger pulse. Specify this value in the units you specify with Width Units.
		DAQmx_Exported_RefTrig_OutputTerm = 0x0590, // Specifies the terminal to which to route the Reference Trigger.
		DAQmx_Exported_StartTrig_OutputTerm = 0x0584, // Specifies the terminal to which to route the Start Trigger.
		DAQmx_Exported_AdvCmpltEvent_OutputTerm = 0x1651, // Specifies the terminal to which to route the Advance Complete Event.
		DAQmx_Exported_AdvCmpltEvent_Delay = 0x1757, // Specifies the output signal delay in periods of the sample clock.
		DAQmx_Exported_AdvCmpltEvent_Pulse_Polarity = 0x1652, // Specifies the polarity of the exported Advance Complete Event.
		DAQmx_Exported_AdvCmpltEvent_Pulse_Width = 0x1654, // Specifies the width of the exported Advance Complete Event pulse.
		DAQmx_Exported_AIHoldCmpltEvent_OutputTerm = 0x18ED, // Specifies the terminal to which to route the AI Hold Complete Event.
		DAQmx_Exported_AIHoldCmpltEvent_PulsePolarity = 0x18EE, // Specifies the polarity of an exported AI Hold Complete Event pulse.
		DAQmx_Exported_ChangeDetectEvent_OutputTerm = 0x2197, // Specifies the terminal to which to route the Change Detection Event.
		DAQmx_Exported_CtrOutEvent_OutputTerm = 0x1717, // Specifies the terminal to which to route the Counter Output Event.
		DAQmx_Exported_CtrOutEvent_OutputBehavior = 0x174F, // Specifies whether the exported Counter Output Event pulses or changes from one state to the other when the counter reaches terminal count.
		DAQmx_Exported_CtrOutEvent_Pulse_Polarity = 0x1718, // Specifies the polarity of the pulses at the output terminal of the counter when Output Behavior is DAQmx_Val_Pulse. NI-DAQmx ignores this property if Output Behavior is DAQmx_Val_Toggle.
		DAQmx_Exported_CtrOutEvent_Toggle_IdleState = 0x186A, // Specifies the initial state of the output terminal of the counter when Output Behavior is DAQmx_Val_Toggle. The terminal enters this state when NI-DAQmx commits the task.
		DAQmx_Exported_WatchdogExpiredEvent_OutputTerm = 0x21AA, // Specifies the terminal  to which to route the Watchdog Timer Expired Event.
	}
	enum {/*Device Attributes}*/
		DAQmx_Dev_ProductType = 0x0631, // Indicates the product name of the device.
		DAQmx_Dev_SerialNum = 0x0632, // Indicates the serial number of the device. This value is zero if the device does not have a serial number.
	}
	enum {/*Read Attributes}*/
		DAQmx_Read_RelativeTo = 0x190A, // Specifies the point in the buffer at which to begin a read operation. If you also specify an offset with Offset, the read operation begins at that offset relative to the point you select with this property. The default value is DAQmx_Val_CurrReadPos unless you configure a Reference Trigger for the task. If you configure a Reference Trigger, the default value is DAQmx_Val_FirstPretrigSamp.
		DAQmx_Read_Offset = 0x190B, // Specifies an offset in samples per channel at which to begin a read operation. This offset is relative to the location you specify with RelativeTo.
		DAQmx_Read_ChannelsToRead = 0x1823, // Specifies a subset of channels in the task from which to read.
		DAQmx_Read_ReadAllAvailSamp = 0x1215, // Specifies whether subsequent read operations read all samples currently available in the buffer or wait for the buffer to become full before reading. NI-DAQmx uses this setting for finite acquisitions and only when the number of samples to read is -1. For continuous acquisitions when the number of samples to read is -1, a read operation always reads all samples currently available in the buffer.
		DAQmx_Read_AutoStart = 0x1826, // Specifies if an NI-DAQmx Read function automatically starts the task  if you did not start the task explicitly by using DAQmxStartTask(). The default value is TRUE. When  an NI-DAQmx Read function starts a finite acquisition task, it also stops the task after reading the last sample.
		DAQmx_Read_OverWrite = 0x1211, // Specifies whether to overwrite samples in the buffer that you have not yet read.
		DAQmx_Read_CurrReadPos = 0x1221, // Indicates in samples per channel the current position in the buffer.
		DAQmx_Read_AvailSampPerChan = 0x1223, // Indicates the number of samples available to read per channel. This value is the same for all channels in the task.
		DAQmx_Read_TotalSampPerChanAcquired = 0x192A, // Indicates the total number of samples acquired by each channel. NI-DAQmx returns a single value because this value is the same for all channels.
		DAQmx_Read_ChangeDetect_HasOverflowed = 0x2194, // Indicates if samples were missed because change detection events occurred faster than the device could handle them.
		DAQmx_Read_RawDataWidth = 0x217A, // Indicates in bytes the size of a raw sample from the task.
		DAQmx_Read_NumChans = 0x217B, // Indicates the number of channels that an NI-DAQmx Read function reads from the task. This value is the number of channels in the task or the number of channels you specify with Channels to Read.
		DAQmx_Read_DigitalLines_BytesPerChan = 0x217C, // Indicates the number of bytes per channel that NI-DAQmx returns in a sample for line-based reads. If a channel has fewer lines than this number, the extra bytes are FALSE.
	}
	enum {/*Switch Channel Attributes}*/
		DAQmx_SwitchChan_Usage = 0x18E4, // Specifies how you can use the channel. Using this property acts as a safety mechanism to prevent you from connecting two source channels, for example.
		DAQmx_SwitchChan_MaxACCarryCurrent = 0x0648, // Indicates in amperes the maximum AC current that the device can carry.
		DAQmx_SwitchChan_MaxACSwitchCurrent = 0x0646, // Indicates in amperes the maximum AC current that the device can switch. This current is always against an RMS voltage level.
		DAQmx_SwitchChan_MaxACCarryPwr = 0x0642, // Indicates in watts the maximum AC power that the device can carry.
		DAQmx_SwitchChan_MaxACSwitchPwr = 0x0644, // Indicates in watts the maximum AC power that the device can switch.
		DAQmx_SwitchChan_MaxDCCarryCurrent = 0x0647, // Indicates in amperes the maximum DC current that the device can carry.
		DAQmx_SwitchChan_MaxDCSwitchCurrent = 0x0645, // Indicates in amperes the maximum DC current that the device can switch. This current is always against a DC voltage level.
		DAQmx_SwitchChan_MaxDCCarryPwr = 0x0643, // Indicates in watts the maximum DC power that the device can carry.
		DAQmx_SwitchChan_MaxDCSwitchPwr = 0x0649, // Indicates in watts the maximum DC power that the device can switch.
		DAQmx_SwitchChan_MaxACVoltage = 0x0651, // Indicates in volts the maximum AC RMS voltage that the device can switch.
		DAQmx_SwitchChan_MaxDCVoltage = 0x0650, // Indicates in volts the maximum DC voltage that the device can switch.
		DAQmx_SwitchChan_WireMode = 0x18E5, // Indicates the number of wires that the channel switches.
		DAQmx_SwitchChan_Bandwidth = 0x0640, // Indicates in Hertz the maximum frequency of a signal that can pass through the switch without significant deterioration.
		DAQmx_SwitchChan_Impedance = 0x0641, // Indicates in ohms the switch impedance. This value is important in the RF domain and should match the impedance of the sources and loads.
	}
	enum {/*Switch Device Attributes}*/
		DAQmx_SwitchDev_SettlingTime = 0x1244, // Specifies in seconds the amount of time to wait for the switch to settle (or debounce). Refer to device documentation for supported settling times.
		DAQmx_SwitchDev_AutoConnAnlgBus = 0x17DA, // Specifies if NI-DAQmx routes multiplexed channels to the analog bus backplane. Only the SCXI-1127 and SCXI-1128 support this property.
		DAQmx_SwitchDev_Settled = 0x1243, // Indicates when Settling Time expires.
		DAQmx_SwitchDev_RelayList = 0x17DC, // Indicates a comma-delimited list of relay names.
		DAQmx_SwitchDev_NumRelays = 0x18E6, // Indicates the number of relays on the device. This value matches the number of relay names in Relay List.
		DAQmx_SwitchDev_SwitchChanList = 0x18E7, // Indicates a comma-delimited list of channel names for the current topology of the device.
		DAQmx_SwitchDev_NumSwitchChans = 0x18E8, // Indicates the number of switch channels for the current topology of the device. This value matches the number of channel names in Switch Channel List.
		DAQmx_SwitchDev_NumRows = 0x18E9, // Indicates the number of rows on a device in a matrix switch topology. Indicates the number of multiplexed channels on a device in a mux topology.
		DAQmx_SwitchDev_NumColumns = 0x18EA, // Indicates the number of columns on a device in a matrix switch topology. This value is always 1 if the device is in a mux topology.
		DAQmx_SwitchDev_Topology = 0x193D, // Indicates the current topology of the device. This value is one of the topology options in DAQmxSwitchSetTopologyAndReset().
	}
	enum {/*Switch Scan Attributes}*/
		DAQmx_SwitchScan_BreakMode = 0x1247, // Specifies the break mode between each entry in a scan list.
		DAQmx_SwitchScan_RepeatMode = 0x1248, // Specifies if the task advances through the scan list multiple times.
		DAQmx_SwitchScan_WaitingForAdv = 0x17D9, // Indicates if the switch hardware is waiting for an  Advance Trigger. If the hardware is waiting, it completed the previous entry in the scan list.
	}
	enum {/*Switch Functions}*/
		DAQmx_Val_Switch_Topology_1127_1_Wire_64x1_Mux = "1127/1-Wire 64x1 Mux", // 1127/1-Wire 64x1 Mux
		DAQmx_Val_Switch_Topology_1127_2_Wire_32x1_Mux = "1127/2-Wire 32x1 Mux", // 1127/2-Wire 32x1 Mux
		DAQmx_Val_Switch_Topology_1127_2_Wire_4x8_Matrix = "1127/2-Wire 4x8 Matrix", // 1127/2-Wire 4x8 Matrix
		DAQmx_Val_Switch_Topology_1127_4_Wire_16x1_Mux = "1127/4-Wire 16x1 Mux", // 1127/4-Wire 16x1 Mux
		DAQmx_Val_Switch_Topology_1127_Independent = "1127/Independent", // 1127/Independent
		DAQmx_Val_Switch_Topology_1128_1_Wire_64x1_Mux = "1128/1-Wire 64x1 Mux", // 1128/1-Wire 64x1 Mux
		DAQmx_Val_Switch_Topology_1128_2_Wire_32x1_Mux = "1128/2-Wire 32x1 Mux", // 1128/2-Wire 32x1 Mux
		DAQmx_Val_Switch_Topology_1128_2_Wire_4x8_Matrix = "1128/2-Wire 4x8 Matrix", // 1128/2-Wire 4x8 Matrix
		DAQmx_Val_Switch_Topology_1128_4_Wire_16x1_Mux = "1128/4-Wire 16x1 Mux", // 1128/4-Wire 16x1 Mux
		DAQmx_Val_Switch_Topology_1128_Independent = "1128/Independent", // 1128/Independent
		DAQmx_Val_Switch_Topology_1129_2_Wire_16x16_Matrix = "1129/2-Wire 16x16 Matrix", // 1129/2-Wire 16x16 Matrix
		DAQmx_Val_Switch_Topology_1129_2_Wire_8x32_Matrix = "1129/2-Wire 8x32 Matrix", // 1129/2-Wire 8x32 Matrix
		DAQmx_Val_Switch_Topology_1129_2_Wire_4x64_Matrix = "1129/2-Wire 4x64 Matrix", // 1129/2-Wire 4x64 Matrix
		DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_8x16_Matrix = "1129/2-Wire Dual 8x16 Matrix", // 1129/2-Wire Dual 8x16 Matrix
		DAQmx_Val_Switch_Topology_1129_2_Wire_Dual_4x32_Matrix = "1129/2-Wire Dual 4x32 Matrix", // 1129/2-Wire Dual 4x32 Matrix
		DAQmx_Val_Switch_Topology_1129_2_Wire_Quad_4x16_Matrix = "1129/2-Wire Quad 4x16 Matrix", // 1129/2-Wire Quad 4x16 Matrix
		DAQmx_Val_Switch_Topology_1130_1_Wire_256x1_Mux = "1130/1-Wire 256x1 Mux", // 1130/1-Wire 256x1 Mux
		DAQmx_Val_Switch_Topology_1130_2_Wire_128x1_Mux = "1130/2-Wire 128x1 Mux", // 1130/2-Wire 128x1 Mux
		DAQmx_Val_Switch_Topology_1130_4_Wire_64x1_Mux = "1130/4-Wire 64x1 Mux", // 1130/4-Wire 64x1 Mux
		DAQmx_Val_Switch_Topology_1130_1_Wire_4x64_Matrix = "1130/1-Wire 4x64 Matrix", // 1130/1-Wire 4x64 Matrix
		DAQmx_Val_Switch_Topology_1130_1_Wire_8x32_Matrix = "1130/1-Wire 8x32 Matrix", // 1130/1-Wire 8x32 Matrix
		DAQmx_Val_Switch_Topology_1130_2_Wire_4x32_Matrix = "1130/2-Wire 4x32 Matrix", // 1130/2-Wire 4x32 Matrix
		DAQmx_Val_Switch_Topology_1130_Independent = "1130/Independent", // 1130/Independent
		DAQmx_Val_Switch_Topology_1160_16_SPDT = "1160/16-SPDT", // 1160/16-SPDT
		DAQmx_Val_Switch_Topology_1161_8_SPDT = "1161/8-SPDT", // 1161/8-SPDT
		DAQmx_Val_Switch_Topology_1163R_Octal_4x1_Mux = "1163R/Octal 4x1 Mux", // 1163R/Octal 4x1 Mux
		DAQmx_Val_Switch_Topology_1166_32_SPDT = "1166/32-SPDT", // 1166/32-SPDT
		DAQmx_Val_Switch_Topology_1167_Independent = "1167/Independent", // 1167/Independent
		DAQmx_Val_Switch_Topology_1190_Quad_4x1_Mux = "1190/Quad 4x1 Mux", // 1190/Quad 4x1 Mux
		DAQmx_Val_Switch_Topology_1191_Quad_4x1_Mux = "1191/Quad 4x1 Mux", // 1191/Quad 4x1 Mux
		DAQmx_Val_Switch_Topology_1192_8_SPDT = "1192/8-SPDT", // 1192/8-SPDT
		DAQmx_Val_Switch_Topology_1193_32x1_Mux = "1193/32x1 Mux", // 1193/32x1 Mux
		DAQmx_Val_Switch_Topology_1193_Dual_16x1_Mux = "1193/Dual 16x1 Mux", // 1193/Dual 16x1 Mux
		DAQmx_Val_Switch_Topology_1193_Quad_8x1_Mux = "1193/Quad 8x1 Mux", // 1193/Quad 8x1 Mux
		DAQmx_Val_Switch_Topology_1193_16x1_Terminated_Mux = "1193/16x1 Terminated Mux", // 1193/16x1 Terminated Mux
		DAQmx_Val_Switch_Topology_1193_Dual_8x1_Terminated_Mux = "1193/Dual 8x1 Terminated Mux", // 1193/Dual 8x1 Terminated Mux
		DAQmx_Val_Switch_Topology_1193_Quad_4x1_Terminated_Mux = "1193/Quad 4x1 Terminated Mux", // 1193/Quad 4x1 Terminated Mux
		DAQmx_Val_Switch_Topology_1193_Independent = "1193/Independent", // 1193/Independent
		DAQmx_Val_Switch_Topology_2529_2_Wire_8x16_Matrix = "2529/2-Wire 8x16 Matrix", // 2529/2-Wire 8x16 Matrix
		DAQmx_Val_Switch_Topology_2529_2_Wire_4x32_Matrix = "2529/2-Wire 4x32 Matrix", // 2529/2-Wire 4x32 Matrix
		DAQmx_Val_Switch_Topology_2529_2_Wire_Dual_4x16_Matrix = "2529/2-Wire Dual 4x16 Matrix", // 2529/2-Wire Dual 4x16 Matrix
		DAQmx_Val_Switch_Topology_2530_1_Wire_128x1_Mux = "2530/1-Wire 128x1 Mux", // 2530/1-Wire 128x1 Mux
		DAQmx_Val_Switch_Topology_2530_2_Wire_64x1_Mux = "2530/2-Wire 64x1 Mux", // 2530/2-Wire 64x1 Mux
		DAQmx_Val_Switch_Topology_2530_4_Wire_32x1_Mux = "2530/4-Wire 32x1 Mux", // 2530/4-Wire 32x1 Mux
		DAQmx_Val_Switch_Topology_2530_1_Wire_4x32_Matrix = "2530/1-Wire 4x32 Matrix", // 2530/1-Wire 4x32 Matrix
		DAQmx_Val_Switch_Topology_2530_1_Wire_8x16_Matrix = "2530/1-Wire 8x16 Matrix", // 2530/1-Wire 8x16 Matrix
		DAQmx_Val_Switch_Topology_2530_2_Wire_4x16_Matrix = "2530/2-Wire 4x16 Matrix", // 2530/2-Wire 4x16 Matrix
		DAQmx_Val_Switch_Topology_2530_Independent = "2530/Independent", // 2530/Independent
		DAQmx_Val_Switch_Topology_2566_16_SPDT = "2566/16-SPDT", // 2566/16-SPDT
		DAQmx_Val_Switch_Topology_2567_Independent = "2567/Independent", // 2567/Independent
		DAQmx_Val_Switch_Topology_2570_40_SPDT = "2570/40-SPDT", // 2570/40-SPDT
		DAQmx_Val_Switch_Topology_2593_16x1_Mux = "2593/16x1 Mux", // 2593/16x1 Mux
		DAQmx_Val_Switch_Topology_2593_Dual_8x1_Mux = "2593/Dual 8x1 Mux", // 2593/Dual 8x1 Mux
		DAQmx_Val_Switch_Topology_2593_8x1_Terminated_Mux = "2593/8x1 Terminated Mux", // 2593/8x1 Terminated Mux
		DAQmx_Val_Switch_Topology_2593_Dual_4x1_Terminated_Mux = "2593/Dual 4x1 Terminated Mux", // 2593/Dual 4x1 Terminated Mux
		DAQmx_Val_Switch_Topology_2593_Independent = "2593/Independent", // 2593/Independent
	}
	enum {/*Scale Attributes}*/
		DAQmx_Scale_Descr = 0x1226, // Specifies a description for the scale.
		DAQmx_Scale_ScaledUnits = 0x191B, // Specifies the units to use for scaled values. You can use an arbitrary string.
		DAQmx_Scale_PreScaledUnits = 0x18F7, // Specifies the units of the values that you want to scale.
		DAQmx_Scale_Type = 0x1929, // Indicates the method or equation form that the custom scale uses.
		DAQmx_Scale_Lin_Slope = 0x1227, // Specifies the slope, m, in the equation y=mx+b.
		DAQmx_Scale_Lin_YIntercept = 0x1228, // Specifies the y-intercept, b, in the equation y=mx+b.
		DAQmx_Scale_Map_ScaledMax = 0x1229, // Specifies the largest value in the range of scaled values. NI-DAQmx maps this value to Pre-Scaled Maximum Value. Reads clip samples that are larger than this value. Writes generate errors for samples that are larger than this value.
		DAQmx_Scale_Map_PreScaledMax = 0x1231, // Specifies the largest value in the range of pre-scaled values. NI-DAQmx maps this value to Scaled Maximum Value.
		DAQmx_Scale_Map_ScaledMin = 0x1230, // Specifies the smallest value in the range of scaled values. NI-DAQmx maps this value to Pre-Scaled Minimum Value. Reads clip samples that are smaller than this value. Writes generate errors for samples that are smaller than this value.
		DAQmx_Scale_Map_PreScaledMin = 0x1232, // Specifies the smallest value in the range of pre-scaled values. NI-DAQmx maps this value to Scaled Minimum Value.
		DAQmx_Scale_Poly_ForwardCoeff = 0x1234, // Specifies an array of coefficients for the polynomial that converts pre-scaled values to scaled values. Each element of the array corresponds to a term of the equation. For example, if index three of the array is 9, the fourth term of the equation is 9x^3.
		DAQmx_Scale_Poly_ReverseCoeff = 0x1235, // Specifies an array of coefficients for the polynomial that converts scaled values to pre-scaled values. Each element of the array corresponds to a term of the equation. For example, if index three of the array is 9, the fourth term of the equation is 9y^3.
		DAQmx_Scale_Table_ScaledVals = 0x1236, // Specifies an array of scaled values. These values map directly to the values in Pre-Scaled Values.
		DAQmx_Scale_Table_PreScaledVals = 0x1237, // Specifies an array of pre-scaled values. These values map directly to the values in Scaled Values.
	}
	enum {/*System Attributes}*/
		DAQmx_Sys_GlobalChans = 0x1265, // Indicates an array that contains the names of all global channels saved on the system.
		DAQmx_Sys_Scales = 0x1266, // Indicates an array that contains the names of all custom scales saved on the system.
		DAQmx_Sys_Tasks = 0x1267, // Indicates an array that contains the names of all tasks saved on the system.
		DAQmx_Sys_DevNames = 0x193B, // Indicates an array that contains the names of all devices installed in the system.
		DAQmx_Sys_NIDAQMajorVersion = 0x1272, // Indicates the major portion of the installed version of NI-DAQ, such as 7 for version 7.0.
		DAQmx_Sys_NIDAQMinorVersion = 0x1923, // Indicates the minor portion of the installed version of NI-DAQ, such as 0 for version 7.0.
	}
	enum {/*Task Attributes}*/
		DAQmx_Task_Name = 0x1276, // Indicates the name of the task.
		DAQmx_Task_Channels = 0x1273, // Indicates the names of all virtual channels in the task.
		DAQmx_Task_NumChans = 0x2181, // Indicates the number of virtual channels in the task.
		DAQmx_Task_Complete = 0x1274, // Indicates whether the task completed execution.
	}
	enum {/*Timing Attributes}*/
		DAQmx_SampQuant_SampMode = 0x1300, // Specifies if a task acquires or generates a finite number of samples or if it continuously acquires or generates samples.
		DAQmx_SampQuant_SampPerChan = 0x1310, // Specifies the number of samples to acquire or generate for each channel if Sample Mode is finite.
		DAQmx_SampTimingType = 0x1347, // Specifies the type of sample timing to use for the task.
		DAQmx_SampClk_Rate = 0x1344, // Specifies the sampling rate in samples per channel per second. If you use an external source for the Sample Clock, set this input to the maximum expected rate of that clock.
		DAQmx_SampClk_Src = 0x1852, // Specifies the terminal of the signal to use as the Sample Clock.
		DAQmx_SampClk_ActiveEdge = 0x1301, // Specifies on which edge of a clock pulse sampling takes place. This property is useful primarily when the signal you use as the Sample Clock is not a periodic clock.
		DAQmx_SampClk_TimebaseDiv = 0x18EB, // Specifies the number of Sample Clock Timebase pulses needed to produce a single Sample Clock pulse.
		DAQmx_SampClk_Timebase_Rate = 0x1303, // Specifies the rate of the Sample Clock Timebase. When the signal you use as the Sample Clock Timebase is not a clock, NI-DAQmx might require the rate to calculate other timing parameters. If this is the case, setting this property to an approximation is preferable to not setting it at all.
		DAQmx_SampClk_Timebase_Src = 0x1308, // Specifies the terminal of the signal to use as the Sample Clock Timebase.
		DAQmx_SampClk_Timebase_ActiveEdge = 0x18EC, // Specifies on which edge to recognize a Sample Clock Timebase pulse. This property is useful primarily when the signal you use as the Sample Clock Timebase is not a periodic clock.
		DAQmx_SampClk_Timebase_MasterTimebaseDiv = 0x1305, // Specifies the number of pulses of the Master Timebase needed to produce a single pulse of the Sample Clock Timebase.
		DAQmx_ChangeDetect_DI_RisingEdgePhysicalChans = 0x2195, // Specifies the names of the digital lines or ports on which to detect rising edges. The lines or ports must be used by virtual channels in the task. You also can specify a string that contains a list or range of digital lines or ports.
		DAQmx_ChangeDetect_DI_FallingEdgePhysicalChans = 0x2196, // Specifies the names of the digital lines or ports on which to detect rising edges. The lines or ports must be used by virtual channels in the task. You also can specify a string that contains a list or range of digital lines or ports.
		DAQmx_OnDemand_SimultaneousAOEnable = 0x21A0, // Specifies whether to update all channels in the task simultaneously, rather than updating channels independently when you write a sample to that channel.
		DAQmx_AIConv_Rate = 0x1848, // Specifies the rate at which to clock the analog-to-digital converter. This clock is specific to the analog input section of an E Series device.
		DAQmx_AIConv_Src = 0x1502, // Specifies the terminal of the signal to use as the AI Convert Clock.
		DAQmx_AIConv_ActiveEdge = 0x1853, // Specifies on which edge of the clock pulse an analog-to-digital conversion takes place.
		DAQmx_AIConv_TimebaseDiv = 0x1335, // Specifies the number of AI Convert Clock Timebase pulses needed to produce a single AI Convert Clock pulse.
		DAQmx_AIConv_Timebase_Src = 0x1339, // Specifies the terminal  of the signal to use as the AI Convert Clock Timebase.
		DAQmx_MasterTimebase_Rate = 0x1495, // Specifies the rate of the Master Timebase.
		DAQmx_MasterTimebase_Src = 0x1343, // Specifies the terminal of the signal to use as the Master Timebase. On an E Series device, you can choose only between the onboard 20MHz Timebase or the RTSI7 terminal.
		DAQmx_DelayFromSampClk_DelayUnits = 0x1304, // Specifies the units of Delay.
		DAQmx_DelayFromSampClk_Delay = 0x1317, // Specifies the amount of time to wait after receiving a Sample Clock edge before beginning to acquire the sample. This value is in the units you specify with Delay Units.
	}
	enum {/*Trigger Attributes}*/
		DAQmx_StartTrig_Type = 0x1393, // Specifies the type of trigger to use to start a task.
		DAQmx_DigEdge_StartTrig_Src = 0x1407, // Specifies the name of a terminal where there is a digital signal to use as the source of the Start Trigger.
		DAQmx_DigEdge_StartTrig_Edge = 0x1404, // Specifies on which edge of a digital pulse to start acquiring or generating samples.
		DAQmx_AnlgEdge_StartTrig_Src = 0x1398, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Start Trigger.
		DAQmx_AnlgEdge_StartTrig_Slope = 0x1397, // Specifies on which slope of the trigger signal to start acquiring or generating samples.
		DAQmx_AnlgEdge_StartTrig_Lvl = 0x1396, // Specifies at what threshold in the units of the measurement or generation to start acquiring or generating samples. Use Slope to specify on which slope to trigger on this threshold.
		DAQmx_AnlgEdge_StartTrig_Hyst = 0x1395, // Specifies a hysteresis level in the units of the measurement or generation. If Slope is DAQmx_Val_RisingSlope, the trigger does not deassert until the source signal passes below  Level minus the hysteresis. If Slope is DAQmx_Val_FallingSlope, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
		DAQmx_AnlgWin_StartTrig_Src = 0x1400, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Start Trigger.
		DAQmx_AnlgWin_StartTrig_When = 0x1401, // Specifies whether the task starts acquiring or generating samples when the signal enters or leaves the window you specify with Bottom and Top.
		DAQmx_AnlgWin_StartTrig_Top = 0x1403, // Specifies the upper limit of the window. Specify this value in the units of the measurement or generation.
		DAQmx_AnlgWin_StartTrig_Btm = 0x1402, // Specifies the lower limit of the window. Specify this value in the units of the measurement or generation.
		DAQmx_StartTrig_Delay = 0x1856, // Specifies an amount of time to wait after the Start Trigger is received before acquiring or generating the first sample. This value is in the units you specify with Delay Units.
		DAQmx_StartTrig_DelayUnits = 0x18C8, // Specifies the units of Delay.
		DAQmx_StartTrig_Retriggerable = 0x190F, // Specifies whether to enable retriggerable counter pulse generation. When you set this property to TRUE, the device generates pulses each time it receives a trigger. The device ignores a trigger if it is in the process of generating pulses.
		DAQmx_RefTrig_Type = 0x1419, // Specifies the type of trigger to use to mark a reference point for the measurement.
		DAQmx_RefTrig_PretrigSamples = 0x1445, // Specifies the minimum number of pretrigger samples to acquire from each channel before recognizing the reference trigger. Post-trigger samples per channel are equal to Samples Per Channel minus the number of pretrigger samples per channel.
		DAQmx_DigEdge_RefTrig_Src = 0x1434, // Specifies the name of a terminal where there is a digital signal to use as the source of the Reference Trigger.
		DAQmx_DigEdge_RefTrig_Edge = 0x1430, // Specifies on what edge of a digital pulse the Reference Trigger occurs.
		DAQmx_AnlgEdge_RefTrig_Src = 0x1424, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Reference Trigger.
		DAQmx_AnlgEdge_RefTrig_Slope = 0x1423, // Specifies on which slope of the source signal the Reference Trigger occurs.
		DAQmx_AnlgEdge_RefTrig_Lvl = 0x1422, // Specifies in the units of the measurement the threshold at which the Reference Trigger occurs.  Use Slope to specify on which slope to trigger at this threshold.
		DAQmx_AnlgEdge_RefTrig_Hyst = 0x1421, // Specifies a hysteresis level in the units of the measurement. If Slope is DAQmx_Val_RisingSlope, the trigger does not deassert until the source signal passes below Level minus the hysteresis. If Slope is DAQmx_Val_FallingSlope, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
		DAQmx_AnlgWin_RefTrig_Src = 0x1426, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the Reference Trigger.
		DAQmx_AnlgWin_RefTrig_When = 0x1427, // Specifies whether the Reference Trigger occurs when the source signal enters the window or when it leaves the window. Use Bottom and Top to specify the window.
		DAQmx_AnlgWin_RefTrig_Top = 0x1429, // Specifies the upper limit of the window. Specify this value in the units of the measurement.
		DAQmx_AnlgWin_RefTrig_Btm = 0x1428, // Specifies the lower limit of the window. Specify this value in the units of the measurement.
		DAQmx_AdvTrig_Type = 0x1365, // Specifies the type of trigger to use to advance to the next entry in a scan list.
		DAQmx_DigEdge_AdvTrig_Src = 0x1362, // Specifies the name of a terminal where there is a digital signal to use as the source of the Advance Trigger.
		DAQmx_DigEdge_AdvTrig_Edge = 0x1360, // Specifies on which edge of a digital signal to advance to the next entry in a scan list.
		DAQmx_PauseTrig_Type = 0x1366, // Specifies the type of trigger to use to pause a task.
		DAQmx_AnlgLvl_PauseTrig_Src = 0x1370, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the trigger.
		DAQmx_AnlgLvl_PauseTrig_When = 0x1371, // Specifies whether the task pauses above or below the threshold you specify with Level.
		DAQmx_AnlgLvl_PauseTrig_Lvl = 0x1369, // Specifies the threshold at which to pause the task. Specify this value in the units of the measurement or generation. Use Pause When to specify whether the task pauses above or below this threshold.
		DAQmx_AnlgLvl_PauseTrig_Hyst = 0x1368, // Specifies a hysteresis level in the units of the measurement or generation. If Pause When is DAQmx_Val_AboveLvl, the trigger does not deassert until the source signal passes below Level minus the hysteresis. If Pause When is DAQmx_Val_BelowLvl, the trigger does not deassert until the source signal passes above Level plus the hysteresis.
		DAQmx_AnlgWin_PauseTrig_Src = 0x1373, // Specifies the name of a virtual channel or terminal where there is an analog signal to use as the source of the trigger.
		DAQmx_AnlgWin_PauseTrig_When = 0x1374, // Specifies whether the task pauses while the trigger signal is inside or outside the window you specify with Bottom and Top.
		DAQmx_AnlgWin_PauseTrig_Top = 0x1376, // Specifies the upper limit of the window. Specify this value in the units of the measurement or generation.
		DAQmx_AnlgWin_PauseTrig_Btm = 0x1375, // Specifies the lower limit of the window. Specify this value in the units of the measurement or generation.
		DAQmx_DigLvl_PauseTrig_Src = 0x1379, // Specifies the name of a terminal where there is a digital signal to use as the source of the Pause Trigger.
		DAQmx_DigLvl_PauseTrig_When = 0x1380, // Specifies whether the task pauses while the signal is high or low.
		DAQmx_ArmStartTrig_Type = 0x1414, // Specifies the type of trigger to use to arm the task for a Start Trigger. If you configure an Arm Start Trigger, the task does not respond to a Start Trigger until the device receives the Arm Start Trigger.
		DAQmx_DigEdge_ArmStartTrig_Src = 0x1417, // Specifies the name of a terminal where there is a digital signal to use as the source of the Arm Start Trigger.
		DAQmx_DigEdge_ArmStartTrig_Edge = 0x1415, // Specifies on which edge of a digital signal to arm the task for a Start Trigger.
	}
	enum {/*Watchdog Attributes}*/
		DAQmx_Watchdog_Timeout = 0x21A9, // Specifies in seconds the amount of time until the watchdog timer expires. A value of -1 means the internal timer never expires. Set this input to -1 if you use an Expiration Trigger to expire the watchdog task.
		DAQmx_WatchdogExpirTrig_Type = 0x21A3, // Specifies the type of trigger to use to expire a watchdog task.
		DAQmx_DigEdge_WatchdogExpirTrig_Src = 0x21A4, // Specifies the name of a terminal where a digital signal exists to use as the source of the Expiration Trigger.
		DAQmx_DigEdge_WatchdogExpirTrig_Edge = 0x21A5, // Specifies on which edge of a digital signal to expire the watchdog task.
		DAQmx_Watchdog_DO_ExpirState = 0x21A7, // Specifies the state to which to set the digital physical channels when the watchdog task expires.  You cannot modify the expiration state of dedicated digital input physical channels.
		DAQmx_Watchdog_HasExpired = 0x21A8, // Indicates if the watchdog timer expired. You can read this property only while the task is running.
	}
	enum {/*Write Attributes}*/
		DAQmx_Write_RelativeTo = 0x190C, // Specifies the point in the buffer at which to write data. If you also specify an offset with Offset, the write operation begins at that offset relative to this point you select with this property.
		DAQmx_Write_Offset = 0x190D, // Specifies in samples per channel an offset at which a write operation begins. This offset is relative to the location you specify with Relative To.
		DAQmx_Write_RegenMode = 0x1453, // Specifies whether to allow NI-DAQmx to generate the same data multiple times.
		DAQmx_Write_CurrWritePos = 0x1458, // Indicates the number of the next sample for the device to generate. This value is identical for all channels in the task.
		DAQmx_Write_SpaceAvail = 0x1460, // Indicates in samples per channel the amount of available space in the buffer.
		DAQmx_Write_TotalSampPerChanGenerated = 0x192B, // Indicates the total number of samples generated by each channel in the task. This value is identical for all channels in the task.
		DAQmx_Write_RawDataWidth = 0x217D, // Indicates in bytes the required size of a raw sample to write to the task.
		DAQmx_Write_NumChans = 0x217E, // Indicates the number of channels that an NI-DAQmx Write function writes to the task. This value is the number of channels in the task.
		DAQmx_Write_DigitalLines_BytesPerChan = 0x217F, // Indicates the number of bytes expected per channel in a sample for line-based writes. If a channel has fewer lines than this number, NI-DAQmx ignores the extra bytes.
	}
	enum {/*Values for the Mode parameter of DAQmxTaskControl}*/
		DAQmx_Val_Task_Start = 0, // Start
		DAQmx_Val_Task_Stop = 1, // Stop
		DAQmx_Val_Task_Verify = 2, // Verify
		DAQmx_Val_Task_Commit = 3, // Commit
		DAQmx_Val_Task_Reserve = 4, // Reserve
		DAQmx_Val_Task_Unreserve = 5, // Unreserve
		DAQmx_Val_Task_Abort = 6, // Abort
	}
	enum {/*Values for the Action parameter of DAQmxControlWatchdogTask}*/
		DAQmx_Val_ResetTimer = 0, // Reset Timer
		DAQmx_Val_ClearExpiration = 1, // Clear Expiration
	}
	enum {/*Values for the Line Grouping parameter of DAQmxCreateDIChan and DAQmxCreateDOChan}*/
		DAQmx_Val_ChanPerLine = 0, // One Channel For Each Line
		DAQmx_Val_ChanForAllLines = 1, // One Channel For All Lines
	}
	enum {/*Values for the Data Layout parameter of DAQmxWriteAnalogF64, DAQmxWriteBinaryI16, DAQmxWriteDigitalU8, DAQmxWriteDigitalU32, DAQmxWriteDigitalLines}*/
		DAQmx_Val_GroupByChannel = 0, // Group by Channel
		DAQmx_Val_GroupByScanNumber = 1, // Group by Scan Number
	}
	enum {/*Values for the Signal Modifiers parameter of DAQmxConnectTerms}*/
		DAQmx_Val_DoNotInvertPolarity = 0, // Do not invert polarity
		DAQmx_Val_InvertPolarity = 1, // Invert polarity
	}
	enum {/*Values for the Action paramter of DAQmxCloseExtCal}*/
		DAQmx_Val_Action_Commit = 0, // Commit
		DAQmx_Val_Action_Cancel = 1, // Cancel
	}
	enum {/*Value set for the Signal ID parameter of DAQmxExportSignal}*/
		DAQmx_Val_AIConvertClock = 12484, // AI Convert Clock
		DAQmx_Val_20MHzTimebaseClock = 12486, // 20MHz Timebase Clock
		DAQmx_Val_SampleClock = 12487, // Sample Clock
		DAQmx_Val_AdvanceTrigger = 12488, // Advance Trigger
		DAQmx_Val_ReferenceTrigger = 12490, // Reference Trigger
		DAQmx_Val_StartTrigger = 12491, // Start Trigger
		DAQmx_Val_AdvCmpltEvent = 12492, // Advance Complete Event
		DAQmx_Val_AIHoldCmpltEvent = 12493, // AI Hold Complete Event
		DAQmx_Val_CounterOutputEvent = 12494, // Counter Output Event
		DAQmx_Val_ChangeDetectionEvent = 12511, // Change Detection Event
		DAQmx_Val_WDTExpiredEvent = 12512, // Watchdog Timer Expired Event
	}
	enum {/*Value set for the output Path Status parameter of DAQmxSwitchFindPath}*/
		DAQmx_Val_PathStatus_Available = 10431, // Path Available
		DAQmx_Val_PathStatus_AlreadyExists = 10432, // Path Already Exists
		DAQmx_Val_PathStatus_Unsupported = 10433, // Path Unsupported
		DAQmx_Val_PathStatus_ChannelInUse = 10434, // Channel In Use
		DAQmx_Val_PathStatus_SourceChannelConflict = 10435, // Channel Source Conflict
		DAQmx_Val_PathStatus_ChannelReservedForRouting = 10436, // Channel Reserved for Routing
	}
	enum {/*Value set for the state parameter of DAQmxSwitchGetSingleRelayPos and DAQmxSwitchGetMultiRelayPos}*/
		DAQmx_Val_Open = 10437, // Open
		DAQmx_Val_Closed = 10438, // Closed
	}
	enum {/*Value for the Terminal Config parameter of DAQmxCreateAIVoltageChan, DAQmxCreateAICurrentChan and DAQmxCreateAIVoltageChanWithExcit}*/
		DAQmx_Val_Cfg_Default = -1, // Default
	}
	enum {/*Value for the Timeout parameter of DAQmxWaitUntilTaskDone}*/
		DAQmx_Val_WaitInfinitely = -1.0
	}
	enum {/*DAQmxReadDigitalLines, DAQmxReadCounterF64, DAQmxReadCounterU32 and DAQmxReadRaw}*/
		DAQmx_Val_Auto = -1, 
	}
	enum {/*Value set AIMeasurementType}*/
		DAQmx_Val_Voltage = 10322, // Voltage
		DAQmx_Val_Current = 10134, // Current
		DAQmx_Val_Voltage_CustomWithExcitation = 10323, // More:Voltage:Custom with Excitation
		DAQmx_Val_Freq_Voltage = 10181, // Frequency
		DAQmx_Val_Resistance = 10278, // Resistance
		DAQmx_Val_Temp_TC = 10303, // Temperature:Thermocouple
		DAQmx_Val_Temp_Thrmstr = 10302, // Temperature:Thermistor
		DAQmx_Val_Temp_RTD = 10301, // Temperature:RTD
		DAQmx_Val_Temp_BuiltInSensor = 10311, // Temperature:Built-in Sensor
		DAQmx_Val_Strain_Gage = 10300, // Strain Gage
		DAQmx_Val_Position_LVDT = 10352, // Position:LVDT
		DAQmx_Val_Position_RVDT = 10353, // Position:RVDT
		DAQmx_Val_Accelerometer = 10356, // Accelerometer
	}
	enum {/*Value set AccelSensitivityUnits1}*/
		DAQmx_Val_mVoltsPerG = 12509, // mVolts/g
		DAQmx_Val_VoltsPerG = 12510, // Volts/g
	}
	enum {/*Value set AccelUnits2}*/
		DAQmx_Val_AccelUnit_g = 10186, // g
	}
	enum {/*Value set AcquisitionType}*/
		DAQmx_Val_FiniteSamps = 10178, // Finite Samples
		DAQmx_Val_ContSamps = 10123, // Continuous Samples
	}
	enum {/*Value set ActiveLevel}*/
		DAQmx_Val_AboveLvl = 10093, // Above Level
		DAQmx_Val_BelowLvl = 10107, // Below Level
	}
	enum {/*Value set AutoZeroType1}*/
		DAQmx_Val_None = 10230, // None
		DAQmx_Val_Once = 10244, // Once
	}
	enum {/*Value set BreakMode}*/
		DAQmx_Val_NoAction = 10227, // No Action
		DAQmx_Val_BreakBeforeMake = 10110, // Break Before Make
	}
	enum {/*Value set BridgeConfiguration1}*/
		DAQmx_Val_FullBridge = 10182, // Full Bridge
		DAQmx_Val_HalfBridge = 10187, // Half Bridge
		DAQmx_Val_QuarterBridge = 10270, // Quarter Bridge
		DAQmx_Val_NoBridge = 10228, // No Bridge
	}
	enum {/*Value set CIMeasurementType}*/
		DAQmx_Val_CountEdges = 10125, // Count Edges
		DAQmx_Val_Freq = 10179, // Frequency
		DAQmx_Val_Period = 10256, // Period
		DAQmx_Val_PulseWidth = 10359, // Pulse Width
		DAQmx_Val_SemiPeriod = 10289, // Semi Period
		DAQmx_Val_Position_AngEncoder = 10360, // Position:Angular Encoder
		DAQmx_Val_Position_LinEncoder = 10361, // Position:Linear Encoder
		DAQmx_Val_TwoEdgeSep = 10267, // Two Edge Separation
	}
	enum {/*Value set CJCSource1}*/
		DAQmx_Val_BuiltIn = 10200, // Built-In
		DAQmx_Val_ConstVal = 10116, // Constant Value
		DAQmx_Val_Chan = 10113, // Channel
	}
	enum {/*Value set COOutputType}*/
		DAQmx_Val_Pulse_Time = 10269, // Pulse:Time
		DAQmx_Val_Pulse_Freq = 10119, // Pulse:Frequency
		DAQmx_Val_Pulse_Ticks = 10268, // Pulse:Ticks
	}
	enum {/*Value set ChannelType}*/
		DAQmx_Val_AI = 10100, // Analog Input
		DAQmx_Val_AO = 10102, // Analog Output
		DAQmx_Val_DI = 10151, // Digital Input
		DAQmx_Val_DO = 10153, // Digital Output
		DAQmx_Val_CI = 10131, // Counter Input
		DAQmx_Val_CO = 10132, // Counter Output
	}
	enum {/*Value set CountDirection1}*/
		DAQmx_Val_CountUp = 10128, // Count Up
		DAQmx_Val_CountDown = 10124, // Count Down
		DAQmx_Val_ExtControlled = 10326, // Externally Controlled
	}
	enum {/*Value set CounterFrequencyMethod}*/
		DAQmx_Val_LowFreq1Ctr = 10105, // Low Frequency with 1 Counter
		DAQmx_Val_HighFreq2Ctr = 10157, // High Frequency with 2 Counters
		DAQmx_Val_LargeRng2Ctr = 10205, // Large Range with 2 Counters
	}
	enum {/*Value set Coupling1}*/
		DAQmx_Val_AC = 10045, // AC
		DAQmx_Val_DC = 10050, // DC
		DAQmx_Val_GND = 10066, // GND
	}
	enum {/*Value set DataTransferMechanism}*/
		DAQmx_Val_DMA = 10054, // DMA
		DAQmx_Val_Interrupts = 10204, // Interrupts
		DAQmx_Val_ProgrammedIO = 10264, // Programmed I/O
	}
	enum {/*Value set DigitalLineState}*/
		DAQmx_Val_High = 10192, // High
		DAQmx_Val_Low = 10214, // Low
		DAQmx_Val_Tristate = 10310, // Tristate
		DAQmx_Val_NoChange = 10160, // No Change
	}
	enum {/*Value set DigitalWidthUnits1}*/
		DAQmx_Val_SampClkPeriods = 10286, // Sample Clock Periods
		DAQmx_Val_Ticks = 10304, // Ticks
	}
	enum {/*Value set Edge1}*/
		DAQmx_Val_Rising = 10280, // Rising
		DAQmx_Val_Falling = 10171, // Falling
	}
	enum {/*Value set EncoderType2}*/
		DAQmx_Val_X1 = 10090, // X1
		DAQmx_Val_X2 = 10091, // X2
		DAQmx_Val_X4 = 10092, // X4
		DAQmx_Val_TwoPulseCounting = 10313, // Two Pulse Counting
	}
	enum {/*Value set EncoderZIndexPhase1}*/
		DAQmx_Val_AHighBHigh = 10040, // A High B High
		DAQmx_Val_AHighBLow = 10041, // A High B Low
		DAQmx_Val_ALowBHigh = 10042, // A Low B High
		DAQmx_Val_ALowBLow = 10043, // A Low B Low
	}
	enum {/*Value set ExcitationSource}*/
		DAQmx_Val_Internal = 10200, // Internal
		DAQmx_Val_External = 10167, // External
	}
	enum {/*Value set ExportActions2}*/
		DAQmx_Val_Pulse = 10265, // Pulse
		DAQmx_Val_Toggle = 10307, // Toggle
		DAQmx_Val_Lvl = 10210, // Level
	}
	enum {/*Value set InputDataTransferCondition}*/
		DAQmx_Val_OnBrdMemMoreThanHalfFull = 10237, // On Board Memory More than Half Full
		DAQmx_Val_OnBrdMemNotEmpty = 10241, // On Board Memory Not Empty
	}
	enum {/*Value set InputTermCfg}*/
		DAQmx_Val_RSE = 10083, // RSE
		DAQmx_Val_NRSE = 10078, // NRSE
		DAQmx_Val_Diff = 10106, // Differential
	}
	enum {/*Value set LVDTSensitivityUnits1}*/
		DAQmx_Val_mVoltsPerVoltPerMillimeter = 12506, // mVolts/Volt/mMeter
		DAQmx_Val_mVoltsPerVoltPerMilliInch = 12505, // mVolts/Volt/0.001 Inch
	}
	enum {/*Value set MIOAIConvertTbSrc}*/
		DAQmx_Val_SameAsSampTimebase = 10284, // Same as Sample Timebase
		DAQmx_Val_SameAsMasterTimebase = 10282, // Same as Master Timebase
	}
	enum {/*Value set OutputDataTransferCondition}*/
		DAQmx_Val_OnBrdMemEmpty = 10235, // On Board Memory Empty
		DAQmx_Val_OnBrdMemHalfFullOrLess = 10239, // On Board Memory Half Full or Less
		DAQmx_Val_OnBrdMemNotFull = 10242, // On Board Memory Less than Full
	}
	enum {/*Value set OverwriteMode1}*/
		DAQmx_Val_OverwriteUnreadSamps = 10252, // Overwrite Unread Samples
		DAQmx_Val_DoNotOverwriteUnreadSamps = 10159, // Do Not Overwrite Unread Samples
	}
	enum {/*Value set Polarity2}*/
		DAQmx_Val_ActiveHigh = 10095, // Active High
		DAQmx_Val_ActiveLow = 10096, // Active Low
	}
	enum {/*Value set RTDType1}*/
		DAQmx_Val_Pt3750 = 12481, // Pt3750
		DAQmx_Val_Pt3851 = 10071, // Pt3851
		DAQmx_Val_Pt3911 = 12482, // Pt3911
		DAQmx_Val_Pt3916 = 10069, // Pt3916
		DAQmx_Val_Pt3920 = 10053, // Pt3920
		DAQmx_Val_Pt3928 = 12483, // Pt3928
		DAQmx_Val_Custom = 10137, // Custom
	}
	enum {/*Value set RVDTSensitivityUnits1}*/
		DAQmx_Val_mVoltsPerVoltPerDegree = 12507, // mVolts/Volt/Degree
		DAQmx_Val_mVoltsPerVoltPerRadian = 12508, // mVolts/Volt/Radian
	}
	enum {/*Value set ReadRelativeTo}*/
		DAQmx_Val_FirstSample = 10424, // First Sample
		DAQmx_Val_CurrReadPos = 10425, // Current Read Position
		DAQmx_Val_RefTrig = 10426, // Reference Trigger
		DAQmx_Val_FirstPretrigSamp = 10427, // First Pretrigger Sample
		DAQmx_Val_MostRecentSamp = 10428, // Most Recent Sample
	}
	enum {/*Value set RegenerationMode1}*/
		DAQmx_Val_AllowRegen = 10097, // Allow Regeneration
		DAQmx_Val_DoNotAllowRegen = 10158, // Do Not Allow Regeneration
	}
	enum {/*Value set ResistanceConfiguration}*/
		DAQmx_Val_2Wire = 2, // 2-Wire
		DAQmx_Val_3Wire = 3, // 3-Wire
		DAQmx_Val_4Wire = 4, // 4-Wire
		DAQmx_Val_5Wire = 5, // 5-Wire
	}
	enum {/*Value set ResolutionType1}*/
		DAQmx_Val_Bits = 10109, // Bits
	}
	enum {/*Value set SampleTimingType}*/
		DAQmx_Val_SampClk = 10388, // Sample Clock
		DAQmx_Val_Handshake = 10389, // Handshake
		DAQmx_Val_Implicit = 10451, // Implicit
		DAQmx_Val_OnDemand = 10390, // On Demand
		DAQmx_Val_ChangeDetection = 12504, // Change Detection
	}
	enum {/*Value set ScaleType}*/
		DAQmx_Val_Linear = 10447, // Linear
		DAQmx_Val_MapRanges = 10448, // Map Ranges
		DAQmx_Val_Polynomial = 10449, // Polynomial
		DAQmx_Val_Table = 10450, // Table
	}
	enum {/*Value set ShuntCalSelect}*/
		DAQmx_Val_A = 12513, // A
		DAQmx_Val_B = 12514, // B
		DAQmx_Val_AandB = 12515, // A and B
	}
	enum {/*Value set Slope1}*/
		DAQmx_Val_RisingSlope = 10280, // Rising
		DAQmx_Val_FallingSlope = 10171, // Falling
	}
	enum {/*Value set StrainGageBridgeType1}*/
		DAQmx_Val_FullBridgeI = 10183, // Full Bridge I
		DAQmx_Val_FullBridgeII = 10184, // Full Bridge II
		DAQmx_Val_FullBridgeIII = 10185, // Full Bridge III
		DAQmx_Val_HalfBridgeI = 10188, // Half Bridge I
		DAQmx_Val_HalfBridgeII = 10189, // Half Bridge II
		DAQmx_Val_QuarterBridgeI = 10271, // Quarter Bridge I
		DAQmx_Val_QuarterBridgeII = 10272, // Quarter Bridge II
	}
	enum {/*Value set SwitchScanRepeatMode}*/
		DAQmx_Val_Finite = 10172, // Finite
		DAQmx_Val_Cont = 10117, // Continuous
	}
	enum {/*Value set SwitchUsageTypes}*/
		DAQmx_Val_Source = 10439, // Source
		DAQmx_Val_Load = 10440, // Load
		DAQmx_Val_ReservedForRouting = 10441, // Reserved for Routing
	}
	enum {/*Value set ThermocoupleType1}*/
		DAQmx_Val_J_Type_TC = 10072, // J
		DAQmx_Val_K_Type_TC = 10073, // K
		DAQmx_Val_N_Type_TC = 10077, // N
		DAQmx_Val_R_Type_TC = 10082, // R
		DAQmx_Val_S_Type_TC = 10085, // S
		DAQmx_Val_T_Type_TC = 10086, // T
		DAQmx_Val_B_Type_TC = 10047, // B
		DAQmx_Val_E_Type_TC = 10055, // E
	}
	enum {/*Value set TriggerType}*/
		DAQmx_Val_AnlgEdge = 10099, // Analog Edge
		DAQmx_Val_DigEdge = 10150, // Digital Edge
		DAQmx_Val_AnlgWin = 10103, // Analog Window
		DAQmx_Val_Software = 10292, // Software
		DAQmx_Val_AnlgLvl = 10101, // Analog Level
		DAQmx_Val_DigLvl = 10152, // Digital Level
	}
	enum {/*Value set UnitsPreScaled}*/
		DAQmx_Val_Volts = 10348, // Volts
		DAQmx_Val_Amps = 10342, // Amps
		DAQmx_Val_DegF = 10144, // Deg F
		DAQmx_Val_DegC = 10143, // Deg C
		DAQmx_Val_DegR = 10145, // Deg R
		DAQmx_Val_Kelvins = 10325, // Kelvins
		DAQmx_Val_Strain = 10299, // Strain
		DAQmx_Val_Ohms = 10384, // Ohms
		DAQmx_Val_Hz = 10373, // Hz
		DAQmx_Val_Seconds = 10364, // Seconds
		DAQmx_Val_Meters = 10219, // Meters
		DAQmx_Val_Inches = 10379, // Inches
		DAQmx_Val_Degrees = 10146, // Degrees
		DAQmx_Val_Radians = 10273, // Radians
		DAQmx_Val_g = 10186, // g
	}
	enum {/*Value set WindowTriggerCondition1}*/
		DAQmx_Val_EnteringWin = 10163, // Entering Window
		DAQmx_Val_LeavingWin = 10208, // Leaving Window
	}
	enum {/*Value set WindowTriggerCondition2}*/
		DAQmx_Val_InsideWin = 10199, // Inside Window
		DAQmx_Val_OutsideWin = 10251, // Outside Window
	}
	enum {/*Value set WriteRelativeTo}*/
		DAQmx_Val_CurrWritePos = 10430, // Current Write Position
	}
}

version (LIVE) unittest {/*basic reading}*/
	import std.math: isNaN;

	TaskHandle task_handle;

	DAQmx.CreateTask ("", &task_handle);
	DAQmx.CreateAIVoltageChan (task_handle, `Dev1/ai0`, "", DAQmx_Val_Cfg_Default, -5, 5, DAQmx_Val_Volts, null);
	DAQmx.CfgSampClkTiming (task_handle, "OnboardClock", 8000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 0);

	int n_samples_read = 0;
	double time_out = 30.0;

	double[1000] samples;

	foreach (sample; samples)
		assert (sample.isNaN);

	DAQmx.StartTask (task_handle);

	DAQmx.ReadAnalogF64 (task_handle, 1000, time_out, DAQmx_Val_GroupByScanNumber, samples.ptr, 1000, &n_samples_read, null);
	assert (n_samples_read == 1000);

	DAQmx.StopTask (task_handle);
	DAQmx.ClearTask (task_handle);

	foreach (sample; samples)
		assert (not (sample.isNaN));
}
version (LIVE) unittest {/*basic writing}*/
	import std.math: abs, sin;
	import evx.constants: π;

	TaskHandle task_handle;

	DAQmx.CreateTask ("", &task_handle);
	DAQmx.CreateAOVoltageChan (task_handle, `Dev1/ao0`, "", -5, 5, DAQmx_Val_Volts, null);
	DAQmx.CfgSampClkTiming (task_handle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 0);

	int n_samples_written = 0;
	double time_out = 30.0;

	double[1000] samples;

	foreach (i, ref sample; samples)
		sample = 5 * abs (sin (π*i/1000.0));

	bool auto_start = false;
	DAQmx.WriteAnalogF64 (task_handle, 1000, auto_start, time_out, DAQmx_Val_GroupByScanNumber, samples.ptr, &n_samples_written, null);

	DAQmx.StartTask (task_handle);

	import evx.units;
	sleep (5.seconds); // attach LED to AO0 and AOGND, it should blink 5 times

	DAQmx.StopTask (task_handle);
	DAQmx.ClearTask (task_handle);
}
//
